#ifndef GRAPH_SOLVER_HPP
#define GRAPH_SOLVER_HPP

#include <vector>
#include <map>
#include <set>
#include <cmath>
#include <algorithm>
#include <queue>
#include <stack>
#include <limits>
#include <iostream>
#include <memory>

// Octomap Includes
#include <octomap/octomap.h>
#include <octomap/OcTree.h>

struct Point3D { 
    double x, y, z, yaw; 
};

struct Edge {
    int u, v;
    double weight;
    // Overload < for sorting (Kruskal's)
    bool operator<(const Edge& other) const { 
        return weight < other.weight; 
    }
};

// Adjacency list: Map<NodeID, List of {NeighborID, Weight}>
typedef std::map<int, std::vector<std::pair<int, double>>> AdjacencyList;

class DisjointSet {
    std::map<int, int> parent;
public:
    void make_set(int v);
    int find(int v);
    void unite(int a, int b);
};

// --- Function Declarations ---

// Calculate Euclidean distance
double get_distance(const Point3D& p1, const Point3D& p2);

// Phase 1: Connect separate buildings into a single graph using Ray-Casting
std::vector<Edge> find_connecting_bridges(
    const std::map<int, Point3D>& all_vertices,
    const std::map<int, int>& vertex_to_graph_id,
    std::shared_ptr<octomap::OcTree> tree
);

// Phase 2: Solve Chinese Postman Problem (CPP)
std::vector<int> solve_chinese_postman_path(
    const std::vector<Edge>& intra, 
    const std::vector<Edge>& bridges, 
    const std::set<int>& vertices
);

#endif // GRAPH_SOLVER_HPP