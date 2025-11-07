#ifndef GRAPH_SOLVER_HPP
#define GRAPH_SOLVER_HPP

#include <vector>
#include <map>
#include <set>
#include <utility>
#include <limits>
#include <cmath>
#include <iostream>
#include <stack>
#include <queue>
#include <fstream>
#include <cstdlib>

#define INF std::numeric_limits<double>::infinity()

// --- Structs and Classes ---

struct Point3D {
    double x, y, z;
};

struct Edge {
    int u, v;
    double weight;
    bool operator<(const Edge& other) const { return weight < other.weight; }
};

class DisjointSet {
    std::map<int, int> parent;
public:
    void make_set(int v);
    int find(int v);
    void unite(int a, int b);
};

using AdjacencyList = std::map<int, std::vector<std::pair<int, double>>>;

// --- Function Declarations ---

double get_distance(const Point3D& p1, const Point3D& p2);

std::vector<Edge> kruskal_mst(std::vector<Edge>& edges, const std::set<int>& vertex_ids);

std::vector<Edge> find_connecting_bridges(
    const std::map<int, Point3D>& all_vertices,
    const std::map<int, int>& vertex_to_graph_id
);

std::map<int, std::pair<double, std::vector<int>>> dijkstra_with_paths(
    int start_node,
    const AdjacencyList& adj,
    const std::set<int>& all_vertex_ids
);

std::pair<double, std::vector<std::pair<int, int>>> find_min_matching_pairs(
    std::vector<int>& odd_nodes,
    const std::map<int, std::map<int, double>>& matching_weights
);

std::vector<int> find_eulerian_circuit(
    int start_node,
    std::map<int, std::multiset<int>>& adj
);

std::vector<int> solve_chinese_postman_path(
    const std::vector<Edge>& intra_graph_edges,
    const std::vector<Edge>& bridge_edges,
    const std::set<int>& all_vertex_ids
);

#endif // GRAPH_SOLVER_HPP
