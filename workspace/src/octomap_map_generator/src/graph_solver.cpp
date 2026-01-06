#include "graph_solver.hpp"

// --- Disjoint Set Implementation ---
void DisjointSet::make_set(int v) { 
    parent[v] = v; 
}

int DisjointSet::find(int v) {
    if (parent.find(v) == parent.end()) make_set(v);
    if (parent[v] == v) return v;
    return parent[v] = find(parent[v]); 
}

void DisjointSet::unite(int a, int b) {
    int rootA = find(a);
    int rootB = find(b);
    if (rootA != rootB) parent[rootB] = rootA;
}

// --- Helper: Distance ---
double get_distance(const Point3D& p1, const Point3D& p2) {
    return std::sqrt(std::pow(p1.x - p2.x, 2) + 
                     std::pow(p1.y - p2.y, 2) + 
                     std::pow(p1.z - p2.z, 2));
}

// --- Phase 1: Bridge Finding with Ray-Casting ---
std::vector<Edge> find_connecting_bridges(
    const std::map<int, Point3D>& all_vertices,
    const std::map<int, int>& vertex_to_graph_id,
    std::shared_ptr<octomap::OcTree> tree
) {
    std::map<std::pair<int, int>, Edge> meta_edges_map;

    // 1. Identify potential bridges between every pair of vertices
    for (auto const& [u, p_u] : all_vertices) {
        int g_u = vertex_to_graph_id.at(u);
        
        for (auto const& [v, p_v] : all_vertices) {
            int g_v = vertex_to_graph_id.at(v);
            
            // Only connect different buildings
            if (g_u >= g_v) continue;

            // --- Ray-Casting Logic ---
            octomap::point3d origin(p_u.x, p_u.y, p_u.z);
            octomap::point3d end(p_v.x, p_v.y, p_v.z);
            octomap::point3d direction = (end - origin).normalize();
            double dist = origin.distance(end);
            octomap::point3d hit;
            
            // If castRay returns true, it HIT an obstacle -> Blocked
            if (tree->castRay(origin, direction, hit, true, dist)) {
                continue; 
            }

            // Keep the shortest safe bridge between building A and B
            if (meta_edges_map.find({g_u, g_v}) == meta_edges_map.end() || 
                dist < meta_edges_map[{g_u, g_v}].weight) {
                meta_edges_map[{g_u, g_v}] = {u, v, dist};
            }
        }
    }

    // 2. Sort potential bridges by weight
    std::vector<Edge> sorted_meta;
    for (auto const& [id, edge] : meta_edges_map) {
        sorted_meta.push_back(edge);
    }
    std::sort(sorted_meta.begin(), sorted_meta.end());

    // 3. Kruskal's Algorithm to select minimum bridges
    DisjointSet ds;
    // Initialize disjoint set for each building ID
    std::set<int> building_ids;
    for (auto const& [v, gid] : vertex_to_graph_id) building_ids.insert(gid);
    for (int gid : building_ids) ds.make_set(gid);

    std::vector<Edge> bridges;
    for (const auto& e : sorted_meta) {
        int rootU = ds.find(vertex_to_graph_id.at(e.u));
        int rootV = ds.find(vertex_to_graph_id.at(e.v));
        
        if (rootU != rootV) {
            bridges.push_back(e);
            ds.unite(rootU, rootV);
        }
    }
    
    std::cout << "Graph Solver: Found " << bridges.size() << " bridges connecting buildings." << std::endl;
    return bridges;
}

// --- Helper: Dijkstra for Matching Odd Nodes ---
// Returns path and cost to all reachable nodes from start
std::pair<std::vector<int>, double> find_shortest_path_dijkstra(
    int start, int goal, const AdjacencyList& adj, const std::set<int>& vertices) 
{
    std::map<int, double> dist;
    std::map<int, int> prev;
    for (int v : vertices) dist[v] = std::numeric_limits<double>::infinity();
    
    dist[start] = 0;
    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<>> pq;
    pq.push({0, start});

    while(!pq.empty()){
        auto [d, u] = pq.top(); pq.pop();
        if (d > dist[u]) continue;
        if (u == goal) break; // Found target

        if (adj.count(u)) {
            for(auto& [v, w] : adj.at(u)) {
                if(dist[u] + w < dist[v]){ 
                    dist[v] = dist[u] + w; 
                    prev[v] = u; 
                    pq.push({dist[v], v}); 
                }
            }
        }
    }

    // Reconstruct Path
    std::vector<int> path;
    if (dist[goal] == std::numeric_limits<double>::infinity()) return {path, -1.0}; // No path

    for (int cur = goal; cur != start; cur = prev[cur]) {
        path.push_back(cur);
    }
    path.push_back(start);
    // Path is currently reversed (goal -> start), but since edges are undirected, order works for insertion
    return {path, dist[goal]};
}

// --- Phase 2: Chinese Postman Solver ---
std::vector<int> solve_chinese_postman_path(
    const std::vector<Edge>& intra, 
    const std::vector<Edge>& bridges, 
    const std::set<int>& vertices
) {
    AdjacencyList adj;
    std::map<int, int> degree;
    // Multiset allows multiple edges between same nodes (needed for Eulerian doubling)
    std::map<int, std::multiset<int>> euler_adj;

    auto add_edge = [&](int u, int v, double w) {
        adj[u].push_back({v, w});
        adj[v].push_back({u, w});
        euler_adj[u].insert(v);
        euler_adj[v].insert(u);
        degree[u]++;
        degree[v]++;
    };

    // 1. Build Base Graph
    for (const auto& e : intra) add_edge(e.u, e.v, e.weight);
    for (const auto& e : bridges) add_edge(e.u, e.v, e.weight);

    // 2. Identify Odd Degree Vertices
    std::vector<int> odds;
    for (int v : vertices) {
        if (degree[v] % 2 != 0) odds.push_back(v);
    }

    std::cout << "CPP Solver: Found " << odds.size() << " odd-degree vertices." << std::endl;

    // 3. Match Odd Vertices (Greedy Approach)
    // We greedily match the first odd node with its closest odd neighbor to make edges even.
    std::set<int> matched;
    for (size_t i = 0; i < odds.size(); ++i) {
        if (matched.count(odds[i])) continue;
        
        int u = odds[i];
        double best_dist = std::numeric_limits<double>::infinity();
        int best_match = -1;
        std::vector<int> best_path;

        // Find closest unmatched odd node
        for (size_t j = i + 1; j < odds.size(); ++j) {
            if (matched.count(odds[j])) continue;
            
            auto result = find_shortest_path_dijkstra(u, odds[j], adj, vertices);
            if (result.second != -1.0 && result.second < best_dist) {
                best_dist = result.second;
                best_match = odds[j];
                best_path = result.first;
            }
        }

        if (best_match != -1) {
            matched.insert(u);
            matched.insert(best_match);
            
            // Add "virtual" edges along the shortest path to double them
            for (size_t k = 0; k < best_path.size() - 1; ++k) {
                int p1 = best_path[k];
                int p2 = best_path[k+1];
                euler_adj[p1].insert(p2);
                euler_adj[p2].insert(p1);
            }
        }
    }

    // 4. Hierholzer's Algorithm for Eulerian Circuit
    std::vector<int> circuit;
    std::stack<int> s;
    
    if (vertices.empty()) return circuit;
    s.push(*vertices.begin()); // Start at any vertex

    while (!s.empty()) {
        int u = s.top();
        if (euler_adj[u].empty()) {
            circuit.push_back(u);
            s.pop();
        } else {
            int v = *euler_adj[u].begin();
            // Remove edge u-v
            euler_adj[u].erase(euler_adj[u].find(v));
            euler_adj[v].erase(euler_adj[v].find(u));
            s.push(v);
        }
    }

    // Circuit is in reverse order, but for a cycle it doesn't strictly matter.
    // However, let's reverse it to match logical flow.
    std::reverse(circuit.begin(), circuit.end());
    
    std::cout << "CPP Solver: Generated path with " << circuit.size() << " steps." << std::endl;
    return circuit;
}