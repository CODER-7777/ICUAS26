#include <chrono>
#include <cmath>
#include <octomap/octomap_types.h>
#include <queue>
#include <cstdlib>
#include <memory>
#include <octomap/OcTree.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/srv/get_octomap.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <algorithm>
const int INF = 1e9; 

using namespace std;
using namespace std::chrono_literals;
// octomap::AbstractOcTree* abstract_tree;
shared_ptr<octomap::OcTree> tree;

struct idx {
  int i;
  int j;
};
typedef struct idx idx;
struct Node {
  octomap::point3d pos;
  vector<idx> index;
  bool visited;
};
typedef struct Node Node;
struct Graph {
  int dim;
  Node *nodes;
};
typedef struct Graph Graph;
Graph *graph;
Graph *g_top;
Graph *g_bottom;

/////////////////////////////////////////////////////////////////////
// struct octomap::point3d {
//   float x;
//   float y;
//   float z;
// };
// typedef struct octomap::point3d octomap::point3d;

bool intersection(octomap::point3d origin, octomap::point3d end) {
  octomap::point3d intersection;
  float distance = origin.distance(end);
  octomap::point3d dir = (end - origin).normalize();
  return tree->castRay(origin, dir, end, true, distance);
}

Graph *create_graph(octomap::point3d start, octomap::point3d end, int dim,
                    float max_dist) {
  float step = (start.distance(end) / 1.414) / (dim - 1);
  int node_rad = max_dist / step;
  Graph *g = new Graph[sizeof(Graph)];
  g->dim = dim;
  g->nodes = new Node[sizeof(Node) * dim * dim];
  for (int i = 0; i < dim; i++) {
    for (int j = 0; j < dim; j++) {
      octomap::point3d pos(i * step, j * step, 0);
      pos += start;
      g->nodes[i * dim + j].pos = pos;
      g->nodes[i * dim + j].visited = false;
    }
  }
  for (int i = 0; i < dim; i++) {
    for (int j = 0; j < dim; j++) {
      for (int k = -node_rad; k < node_rad; k++) {
        if (k == 0)
          continue;
        for (int l = -node_rad; l < node_rad; l++) {
          if (l == 0)
            continue;
          if ((i + k) * dim + j + l < 0 || (i + k) * dim + j + l >= dim * dim)
            continue;
          auto self_pos = g->nodes[i * dim + j].pos;
          auto target_pos = g->nodes[(i + k) * dim + j + l].pos;
          bool intersecting = intersection(self_pos, target_pos);
          if (!intersecting && self_pos.distance(target_pos) < max_dist) {
            idx id;
            id.i = i + k;
            id.j = j + l;
            g->nodes[i * dim + j].index.push_back(id);
          }
        }
      }
    }
  }
  cout << "Graph Created" << endl;
  return g;
}
void print_graph(Graph *g) {
  int dim = g->dim;
  for (int i = 0; i < dim; i++) {
    for (int j = 0; j < dim; j++) {
      cout << endl << "For pos" << g->nodes[i * dim + j].pos << endl;
      vector<idx> indexes = g->nodes[i * dim + j].index;
      for (size_t k = 0; k < indexes.size(); k++) {
        cout << k + 1 << "->" << g->nodes[indexes[k].i * dim + indexes[k].j].pos
             << endl;
      }
    }
  }
}
/////////////////////////////////////////////////////////////////////

class OctomapExtractor : public rclcpp::Node {
public:
  OctomapExtractor() : Node("octomap_extractor") {
    RCLCPP_INFO(this->get_logger(), "Creating Client...");
    rclcpp::Client<octomap_msgs::srv::GetOctomap>::SharedPtr client =
        this->create_client<octomap_msgs::srv::GetOctomap>("octomap_binary");
    RCLCPP_INFO(this->get_logger(), "Client Created");

    while (!client->wait_for_service(10s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Interrupted while waiting for the service. Exiting.");
      }
      RCLCPP_INFO(this->get_logger(), "Waiting for octomap_binary service...");
    }
    RCLCPP_INFO(this->get_logger(), "Service Available");

    auto request = std::make_shared<octomap_msgs::srv::GetOctomap::Request>();
    auto result = client->async_send_request(request);
    RCLCPP_INFO(this->get_logger(), "Request sent. Waiting for response...");

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                           result) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "Request Successful");
      processOctomap(result.get()->map);
      octomap::point3d start(0, 0, 5);
      octomap::point3d end(120, 120, 5);
      graph = create_graph(start, end, 30, 40);

      // octomap::point3d start1(0, 0, 8);
      // octomap::point3d end1(120, 120, 8);
      // g_top = create_graph(start1, end1, 30, 15);

      // octomap::point3d start2(0, 0, 2);
      // octomap::point3d end2(120, 120, 2);
      // g_bottom = create_graph(start2, end2, 30, 15);

      print_graph(graph);
      RCLCPP_INFO(this->get_logger(),"Graph Created!");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Service call failed or timed out.");
    }
  }

private:
  void processOctomap(const octomap_msgs::msg::Octomap &msg) {
    if (msg.binary) {
      tree = std::make_shared<octomap::OcTree>(msg.resolution);
      octomap::AbstractOcTree *abstract_tree =
          octomap_msgs::binaryMsgToMap(msg);
      if (abstract_tree) {
        tree.reset(dynamic_cast<octomap::OcTree *>(abstract_tree));
        if (tree) {
          RCLCPP_INFO(this->get_logger(),
                      "Successfully loaded octomap with resolution: %f",
                      tree->getResolution());
          octomap::point3d origin(10, 51, 1); // Ray origin
          // octomap::point3d origin(0.0, 0.0, 0.0);  // Ray origin
          octomap::point3d direction(1, 0, 0); // Ray direction (x-axis)
          octomap::point3d end(40, 51, 1); // To store the intersection point
          // if (tree->castRay(origin, direction, end, true, 10)) {
          if (intersection(origin, end)) {
            RCLCPP_INFO(this->get_logger(), "Ray hit at: (%.2f, %.2f, %.2f)",
                        end.x(), end.y(), end.z());
          } else {
            RCLCPP_INFO(this->get_logger(),
                        "Ray did not hit any occupied cell.");
          }
        } else {
          RCLCPP_ERROR(this->get_logger(), "Failed to cast octomap to OcTree");
        }
      } else {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to convert Octomap message to AbstractOcTree");
      }
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Received non-binary octomap, skipping...");
    }
  }
  //    rclcpp::Client<octomap_msgs::srv::GetOctomap>::SharedPtr client_;
};


bool valid(int x, int y, int N) {
    return (x >= 0 && x < N && y >= 0 && y < N);
}

vector<idx> bfs_shortest_path(idx start, idx end){
    int n=graph->dim;
    vector<vector<int>> dist(n,vector<int>(n,INF));
    queue<idx> q;
    q.push(start);
    vector<vector<idx>> parent(n, vector<idx>(n, {-1,-1}));
    while(!q.empty()){
        auto cur=q.front();
        q.pop();
        if(cur.i==end.i && cur.j==end.j) break;
 

        for(auto it: graph->nodes[cur.i*n+cur.j].index){
            if(dist[it.i][it.j]==INF){
                q.push(it);
                dist[it.i][it.j]=dist[cur.i][cur.j]+1;
                parent[it.i][it.j]=cur;
            }
        }
    }
    if(dist[end.i][end.j] == INF) return {};
    vector<idx> path;
    idx cur=end;
    while(true){
        path.push_back(cur);
        if(cur.i==start.i && cur.j==start.j) break;
        idx prev=parent[cur.i][cur.j];
        cur=prev;
    }
    
    reverse(path.begin(), path.end()); 
    return path;
}

vector<idx> bfspoints( idx start, idx end, int maxDist){
    vector<idx> path=bfs_shortest_path(start, end);
    idx cur=start;
    int dim=graph->dim;
    vector<idx> points;
    bool moved=false;
    while(!(cur.i == end.i && cur.j == end.j)){
        for(int i=path.size()-1;i>=0;i--){
            octomap::point3d origin=graph->nodes[cur.i*dim+cur.j].pos;
            octomap::point3d fin=graph->nodes[path[i].i*dim+path[i].j].pos;
            bool connected=intersection(origin,fin);
            if(connected && origin.distance(fin)<maxDist){
                cur=path[i];
                points.push_back(path[i]);
                moved=true;
                break;

            }
        }
        if(!moved){
            break;
        }
    }

    return points;
} 


// class SwarmController : public rclcpp::Node {
// public:
//   SwarmController() : Node("SwarmController") {
//     int N = 5;
//     cout << "Controller running..." << endl;
//     idx first_drone; // We need to determine this from the start position
//     first_drone.i = 0;
//     first_drone.j = 0;
//     vector<idx> path;
//     path.push_back(first_drone);
//     for (int k = 1; k <= N; k++) {
//       takeoff_to_position(5, k, 20);
//     }
//     this_thread::sleep_for(21s);
//     traverse(path, 0, 1);
//     dfs(first_drone, 0, N, path);
//   }
//
// private:
//   void dfs(idx curr, int depth, int N, vector<idx> path) {
//     if (depth + 1 > N) {
//       graph->nodes[curr.i * dim + curr.j].visited = true;
//       path.pop_back();
//       return;
//     }
//     int dim = graph->dim;
//     graph->nodes[curr.i * dim + curr.j].visited = true;
//     vector<idx> possibilities = graph->nodes[curr.i * dim + curr.j].index;
//
//     for (int k = 0; k < possibilities.size() || k > 5; k++) {
//       idx poss = possibilities[k];
//       if (!graph->nodes[poss.i * dim + poss.j].visited &&
//           idx_to_pos(poss).distance(idx_to_pos(curr)) > 20) {
//         path.push_back(poss);
//         traverse(path, depth, depth + 1);
//         dfs(possibilities[k], depth + 1, N, path);
//       }
//     }
//     path.pop_back();
//     octomap::point3d offset(0, 0, 0.1 * (depth + 1));
//     go_to_pos(idx_to_pos(path.back()) + offset, depth + 1, 20, false);
//   }
//   void traverse(vector<idx> path, int pos, int drone_no) {
//     octomap::point3d offset(0, 0, 0.1);
//     for (int k = 0; k < pos; k++) {
//       go_to_pos(idx_to_pos(path[k]) + offset, drone_no, 20, false);
//       this_thread::sleep_for(21s);
//     }
//     go_to_pos(idx_to_pos(path[pos]), drone_no, 20, false);
//     this_thread::sleep_for(21s);
//   }
//   octomap::point3d idx_to_pos(idx index) {
//     int dim = graph->dim;
//     return graph->nodes[index.i * dim + index.j].pos;
//   }
//   void takeoff_to_position(float h, int drone_no, int time) {
//     RCLCPP_INFO(this->get_logger(), "Creating Client...");
//     char service_name[20];
//     sprintf(service_name, "cf_%d/takeoff", drone_no);
//     // rclcpp::Client<crazyflie_interfaces::srv::Takeoff>::SharedPtr client =
//     // this->create_client<crazyflie_interfaces::srv::Takeoff>("cf_1/takeoff");
//     rclcpp::Client<crazyflie_interfaces::srv::Takeoff>::SharedPtr client =
//         this->create_client<crazyflie_interfaces::srv::Takeoff>(service_name);
//     RCLCPP_INFO(this->get_logger(), "Client Created");
//     while (!client->wait_for_service(10s)) {
//       if (!rclcpp::ok()) {
//         RCLCPP_ERROR(this->get_logger(),
//                      "Interrupted while waiting for the service. Exiting.");
//       }
//       RCLCPP_INFO(this->get_logger(), "Waiting for takeoff service...");
//     }
//     RCLCPP_INFO(this->get_logger(), "takeoff service availale...");
//
//     auto request =
//         std::make_shared<crazyflie_interfaces::srv::Takeoff::Request>();
//     request->group_mask = 1;
//     request->height = 5.0;
//     request->duration.sec = 50;
//     request->duration.nanosec = 0;
//     auto result = client->async_send_request(request);
//     RCLCPP_INFO(this->get_logger(), "Request sent. Waiting for response...");
//
//     if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
//                                            result) ==
//         rclcpp::FutureReturnCode::SUCCESS) {
//       RCLCPP_INFO(this->get_logger(), "TakeOff Successful");
//     } else {
//       RCLCPP_ERROR(this->get_logger(),
//                    "Takeoff Service call failed or timed out.");
//     }
//   }
//   void go_to_pos(octomap::point3d dest, int drone_no, int time, bool relative) {
//     RCLCPP_INFO(this->get_logger(), "Creating Client...");
//     char service_name[20];
//     sprintf(service_name, "cf_%d/go_to", drone_no);
//     // rclcpp::Client<crazyflie_interfaces::srv::Takeoff>::SharedPtr client =
//     // this->create_client<crazyflie_interfaces::srv::Takeoff>("cf_1/takeoff");
//     rclcpp::Client<crazyflie_interfaces::srv::GoTo>::SharedPtr client =
//         this->create_client<crazyflie_interfaces::srv::GoTo>(service_name);
//     RCLCPP_INFO(this->get_logger(), "Client Created");
//     while (!client->wait_for_service(10s)) {
//       if (!rclcpp::ok()) {
//         RCLCPP_ERROR(this->get_logger(),
//                      "Interrupted while waiting for the service. Exiting.");
//       }
//       RCLCPP_INFO(this->get_logger(), "Waiting for Go to service...");
//     }
//     RCLCPP_INFO(this->get_logger(), "go to service availale...");
//
//     auto request = std::make_shared<crazyflie_interfaces::srv::GoTo::Request>();
//
//     request->goal.x = dest.x();
//     request->goal.y = dest.y();
//     request->goal.z = dest.z();
//
//     request->duration.sec = time;
//     request->relative = relative;
//     request->duration.nanosec = 0;
//     auto result = client->async_send_request(request);
//     RCLCPP_INFO(this->get_logger(), "Request sent. Waiting for response...");
//
//     if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
//                                            result) ==
//         rclcpp::FutureReturnCode::SUCCESS) {
//       RCLCPP_INFO(this->get_logger(), "Go to Successful");
//     } else {
//       RCLCPP_ERROR(this->get_logger(),
//                    "Go to Service call failed or timed out.");
//     }
//   }
// };
//
int main(int argc, char **argv) {
  cout << "Starting node..." << endl;
  rclcpp::init(argc, argv);
  auto node = make_shared<OctomapExtractor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

/*
To build and run this node:
1. Ensure your ROS 2 workspace is set up properly.
2. Place this file inside a ROS 2 package.
3. Modify CMakeLists.txt:
    add_executable(octomap_extractor src/octomap_extractor.cpp)
    ament_target_dependencies(octomap_extractor rclcpp octomap_msgs octomap)
4. Modify package.xml:
    <depend>rclcpp</depend>
    <depend>octomap_msgs</depend>
    <depend>octomap</depend>
5. Build with colcon:
    colcon build --packages-select your_package_name
6. Source the setup file:
    source install/setup.bash
7. Run the node:
    ros2 run your_package_name octomap_extractor
*/
// void travel(){
//     int dim = graph->dim;
//     takeoff_to_position(5,1,10);
//     this_thread::sleep_for(15s);
//     int k = 50;
//     idx curr;
//     curr.i=0;
//     curr.j=0;
//     while (k--){
//         int last =  graph->nodes[curr.i*dim + curr.j].index.size();
//         idx new_pos = graph->nodes[curr.i*dim + curr.j].index[last-1];
//         go_to_pos(graph->nodes[new_pos.i*dim + new_pos.j].pos,1,15,false);
//         curr = new_pos;
//         this_thread::sleep_for(20s);
//     }
// }
// void travel(){
//     int di=graph->dim;
//     auto curr=graph->nodes[0 * di + 0].pos;
//     int x=curr.x();
//     int y=curr.y();
//     int z=curr.z();
//     goto_pos(x,y,z,20);
//     this_thread::sleep_for(30s);
//     int k=50;
//     vector<idx>indexes=curr.index;
//     while(k--){
//         auto curr=graph->nodes[indexes[indexes.size()-1].i * di +
//         indexes[indexes.size()-1].j]; if(curr.visited){
//             curr=graph->nodes[indexes[indexes.size()-2].i * di +
//             indexes[indexes.size()-2].j];
//         }
//         curr.visited=true;
//         auto position=curr.pos;
//         int x1=position.x();
//         int y1=position.y();
//         int z1=position.z();
//         goto_pos(x1,y1,z1,10);
//         indexes = curr.index;
//         this_thread::sleep_for(30s);
//     }
// }
