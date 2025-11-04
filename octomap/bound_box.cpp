// mkdir build
// cmake .. (only once)
// make
// cd ..
// ./build/bound_box city_1_binvox.bt <height>

#include <octomap/octomap.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <string>
#include <stdexcept>
#include <cmath>
#include <unordered_set>
using namespace std;
using namespace cv;
using namespace octomap;

struct PointHash {
    size_t operator()(const Point2i& p) const {
        auto hash1 = hash<int>()(p.x);
        auto hash2 = hash<int>()(p.y);
        return hash1 ^ (hash2 << 1);
    }
};

int main(int argc, char** argv) {
if (argc < 3) { 
        cerr << "Usage: ./extract_bounding_boxes <map.binvox.bt> <height_z>" << endl;
        cerr << "Example: ./extract_bounding_boxes my_map.bt 1.5" << endl;
        return -1;
    }

    string map_file = argv[1];
    double z_target; 
    try {
        z_target = stod(argv[2]);
    } catch (const invalid_argument& e) {
        cerr << "Error: Invalid height. Please provide a number for <height_z>." << endl;
        return -1;
    }

    OcTree tree(map_file);

    double resolution = tree.getResolution();
    double min_x, min_y, min_z;
    double max_x, max_y, max_z;
    tree.getMetricMin(min_x, min_y, min_z);
    tree.getMetricMax(max_x, max_y, max_z);

    double half_res = resolution / 2.0;
    point3d slice_min(min_x, min_y, z_target - half_res);
    point3d slice_max(max_x, max_y, z_target + half_res);

    vector<Point2i> occupied_points;
    
    for (auto it = tree.begin_leafs_bbx(slice_min, slice_max), end = tree.end_leafs_bbx(); it != end; ++it) {
        
        if (tree.isNodeOccupied(*it)) {
            double x = it.getX();
            double y = it.getY();
            
            int gx = static_cast<int>(round(x / resolution));
            int gy = static_cast<int>(round(y / resolution));
            occupied_points.emplace_back(gx, gy);
        }
    }

    cout<< "Total occupied voxels *at target height*: " << occupied_points.size() << endl;

    if (occupied_points.empty()) {
        cerr << "No occupied voxels found at this height!" << endl;
        return 0;
    }

    unordered_set<Point2i, PointHash> occupied_set(occupied_points.begin(), occupied_points.end());

    vector<vector<Point2i>> safe_point_per_obstacle;
    safe_point_per_obstacle.reserve(occupied_points.size());
    unordered_set<Point2i, PointHash> all_unique_safe_points;
    
    for(const auto& obstacle_pt : occupied_points) {
        vector<Point2i> current_safe_points;
        Point2i neighbors[4] = {
            Point2i(obstacle_pt.x, obstacle_pt.y + 1), Point2i(obstacle_pt.x, obstacle_pt.y - 1),
            Point2i(obstacle_pt.x + 1, obstacle_pt.y), Point2i(obstacle_pt.x - 1, obstacle_pt.y)
        };
        for(const auto& neighbor : neighbors) {
            if(occupied_set.count(neighbor) == 0) {
                current_safe_points.push_back(neighbor);
                all_unique_safe_points.insert(neighbor);
            }
        }
        safe_point_per_obstacle.push_back(current_safe_points);
    }
    cout<<"Total unique safe points found: "<<all_unique_safe_points.size()<<endl;

    int min_gx = std::numeric_limits<int>::max();
    int max_gx = std::numeric_limits<int>::min();
    int min_gy = std::numeric_limits<int>::max();
    int max_gy = std::numeric_limits<int>::min();

    for (const auto& pt : occupied_points) {
        min_gx = std::min(min_gx, pt.x);
        max_gx = std::max(max_gx, pt.x);
        min_gy = std::min(min_gy, pt.y);
        max_gy = std::max(max_gy, pt.y);
    }

    const int CELL_SIZE = 5;
    int img_width = (max_gx - min_gx + 1) * CELL_SIZE;
    int img_height = (max_gy - min_gy + 1) * CELL_SIZE;

    Mat map_image_color = Mat::zeros(img_height, img_width, CV_8UC3);
    
    Mat map_image_binary = Mat::zeros(img_height, img_width, CV_8U);

    for (const auto& pt : occupied_points) {
        int px = (pt.x - min_gx) * CELL_SIZE;
        int py = (pt.y - min_gy) * CELL_SIZE;
        
        rectangle(map_image_color, Point(px, py), Point(px + CELL_SIZE, py + CELL_SIZE), Scalar(0, 0, 255), FILLED);
        
        rectangle(map_image_binary, Point(px, py), Point(px + CELL_SIZE, py + CELL_SIZE), Scalar(255), FILLED);
    }

    cout << "\n--- Finding Building Outlines ---" << endl;
    
    Mat labels;
    int num_labels = connectedComponents(map_image_binary, labels, 4);

    cout << "Found " << num_labels - 1 << " distinct obstacles (clusters)." << endl;

    vector<vector<Point>> waypoints_per_building; 

    for (int i = 1; i < num_labels; ++i) {
        Mat mask = (labels == i);
        
        vector<Point> cluster_pixels;
        findNonZero(mask, cluster_pixels);

        if (cluster_pixels.size() < 5 && cluster_pixels.size() > 50) continue;
        vector<Point> hull;
        convexHull(cluster_pixels, hull);

        
        waypoints_per_building.push_back(hull);

        vector<vector<Point>> hull_to_draw = {hull};
        polylines(map_image_color, hull_to_draw, true, Scalar(0, 255, 255), 2);
    }

    cout << "Displaying map. Red=Obstacle, Yellow=Building Outline" << endl;
    imwrite("map_visualization.png", map_image_color);

    return 0;
}