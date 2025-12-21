#include "graph_solver.hpp"
#include <algorithm>

// --------------------
// Basic Helper Methods
// --------------------

double get_distance(const Point3D& p1, const Point3D& p2) {
    return std::sqrt(
        std::pow(p1.x - p2.x, 2) +
        std::pow(p1.y - p2.y, 2) +
        std::pow(p1.z - p2.z, 2)
    );
}

void DisjointSet::make_set(int v) { parent[v] = v; }

int DisjointSet::find(int v) {
    if (parent.find(v) == parent.end()) make_set(v);
    if (v == parent[v]) return v;
    return parent[v] = find(parent[v]); // Path compression
}

void DisjointSet::unite(int a, int b) {
    a = find(a);
    b = find(b);
    if (a != b) parent[b] = a;
}

// --------------------
// Kruskal's MST
// --------------------
std::vector<Edge> kruskal_mst(std::vector<Edge>& edges, const std::set<int>& vertex_ids) {
    std::sort(edges.begin(), edges.end());
    DisjointSet ds;
    for (int v : vertex_ids) ds.make_set(v);

    std::vector<Edge> mst_edges;
    for (const auto& e : edges) {
        if (ds.find(e.u) != ds.find(e.v)) {
            mst_edges.push_back(e);
            ds.unite(e.u, e.v);
        }
    }
    return mst_edges;
}

// --------------------
// Find Connecting Bridges (Meta-Graph)
// --------------------
std::vector<Edge> find_connecting_bridges(
    const std::map<int, Point3D>& all_vertices,
    const std::map<int, int>& vertex_to_graph_id
) {
    std::map<std::pair<int, int>, Edge> meta_graph_edges;

    for (const auto& [u, p_u] : all_vertices) {
        int g_u = vertex_to_graph_id.at(u);
        for (const auto& [v, p_v] : all_vertices) {
            int g_v = vertex_to_graph_id.at(v);
            if (g_u >= g_v) continue;

            double dist = get_distance(p_u, p_v);
            std::pair<int, int> id = {g_u, g_v};
            if (meta_graph_edges.find(id) == meta_graph_edges.end() ||
                dist < meta_graph_edges[id].weight) {
                meta_graph_edges[id] = {u, v, dist};
            }
        }
    }

    std::vector<Edge> meta_edges;
    std::set<int> meta_vertices;
    for (auto& [id, edge] : meta_graph_edges) {
        meta_edges.push_back({id.first, id.second, edge.weight});
        meta_vertices.insert(id.first);
        meta_vertices.insert(id.second);
    }

    std::vector<Edge> meta_mst = kruskal_mst(meta_edges, meta_vertices);
    std::vector<Edge> result;
    for (auto& e : meta_mst)
        result.push_back(meta_graph_edges[{e.u, e.v}]);

    std::cout << "Phase 1 Complete: Found " << result.size() << " bridges.\n";
    return result;
}

// --------------------
// Dijkstra with Paths
// --------------------
std::map<int, std::pair<double, std::vector<int>>> dijkstra_with_paths(
    int start_node, const AdjacencyList& adj, const std::set<int>& vertices) {
    
    std::map<int, double> dist;
    std::map<int, int> prev;
    for (int v : vertices) dist[v] = INF;
    dist[start_node] = 0;

    using Pair = std::pair<double, int>;
    std::priority_queue<Pair, std::vector<Pair>, std::greater<Pair>> pq;
    pq.push({0, start_node});

    while (!pq.empty()) {
        auto [d, u] = pq.top(); pq.pop();
        if (d > dist[u]) continue;
        if (!adj.count(u)) continue;
        for (auto [v, w] : adj.at(u)) {
            if (dist[u] + w < dist[v]) {
                dist[v] = dist[u] + w;
                prev[v] = u;
                pq.push({dist[v], v});
            }
        }
    }

    std::map<int, std::pair<double, std::vector<int>>> result;
    for (int v : vertices) {
        result[v].first = dist[v];
        if (dist[v] == INF) continue;

        std::vector<int> path;
        for (int cur = v; cur != start_node; cur = prev[cur]) {
            path.push_back(cur);
            if (!prev.count(cur)) break;
        }
        path.push_back(start_node);
        std::reverse(path.begin(), path.end());
        result[v].second = path;
    }
    return result;
}

// --------------------
// Recursive Matching
// --------------------
std::pair<double, std::vector<std::pair<int, int>>> find_min_matching_pairs(
    std::vector<int>& odd_nodes,
    const std::map<int, std::map<int, double>>& weights) {
    
    if (odd_nodes.empty()) return {0, {}};
    int u = odd_nodes.back();
    odd_nodes.pop_back();

    double min_cost = INF;
    std::vector<std::pair<int, int>> best;
    for (size_t i = 0; i < odd_nodes.size(); ++i) {
        int v = odd_nodes[i];
        std::vector<int> rest;
        for (size_t j = 0; j < odd_nodes.size(); ++j)
            if (i != j) rest.push_back(odd_nodes[j]);
        
        auto sub = find_min_matching_pairs(rest, weights);
        double cost = weights.at(u).at(v) + sub.first;
        if (cost < min_cost) {
            min_cost = cost;
            best = sub.second;
            best.push_back({u, v});
        }
    }
    odd_nodes.push_back(u);
    return {min_cost, best};
}

// --------------------
// Hierholzer’s Algorithm
// --------------------
std::vector<int> find_eulerian_circuit(int start, std::map<int, std::multiset<int>>& adj) {
    std::vector<int> circuit;
    std::stack<int> s;
    s.push(start);

    while (!s.empty()) {
        int u = s.top();
        if (adj[u].empty()) {
            circuit.push_back(u);
            s.pop();
        } else {
            int v = *adj[u].begin();
            adj[u].erase(adj[u].find(v));
            adj[v].erase(adj[v].find(u));
            s.push(v);
        }
    }
    std::reverse(circuit.begin(), circuit.end());
    return circuit;
}

// --------------------
// Chinese Postman Solver
// --------------------
std::vector<int> solve_chinese_postman_path(
    const std::vector<Edge>& intra, const std::vector<Edge>& bridges, const std::set<int>& vertices) {

    AdjacencyList graph;
    std::map<int, int> deg;
    double w_base = 0;

    auto add = [&](const Edge& e) {
        graph[e.u].push_back({e.v, e.weight});
        graph[e.v].push_back({e.u, e.weight});
        deg[e.u]++; deg[e.v]++; w_base += e.weight;
    };

    for (auto& e : intra) add(e);
    for (auto& e : bridges) add(e);

    std::vector<int> odds;
    for (int v : vertices)
        if (deg[v] % 2) odds.push_back(v);

    std::cout << "Phase 2: Found " << odds.size() << " odd vertices.\n";

    std::map<int, std::multiset<int>> aug;
    for (auto& e : intra) { aug[e.u].insert(e.v); aug[e.v].insert(e.u); }
    for (auto& e : bridges) { aug[e.u].insert(e.v); aug[e.v].insert(e.u); }

    if (odds.empty())
        return find_eulerian_circuit(*vertices.begin(), aug);

    std::map<int, std::map<int, std::vector<int>>> paths;
    std::ofstream problem("matching_problem.txt");
    for (size_t i = 0; i < odds.size(); ++i) {
        int u = odds[i];
        auto paths_u = dijkstra_with_paths(u, graph, vertices);
        for (size_t j = i + 1; j < odds.size(); ++j) {
            int v = odds[j];
            paths[u][v] = paths_u[v].second;
            paths[v][u] = paths_u[v].second;
            problem << u << " " << v << " " << paths_u[v].first << "\n";
        }
    }
    problem.close();

    std::cout << "Phase 2: Calling Python solver...\n";
    system("python3 solve_matching.py");

    std::vector<std::pair<int, int>> pairs;
    std::ifstream sol("matching_solution.txt");
    int u, v;
    while (sol >> u >> v) pairs.push_back({u, v});
    sol.close();

    for (auto& p : pairs) {
        auto path = paths[p.first][p.second];
        for (size_t i = 0; i + 1 < path.size(); ++i) {
            aug[path[i]].insert(path[i + 1]);
            aug[path[i + 1]].insert(path[i]);
        }
    }

    std::cout << "Phase 2: Finding Eulerian circuit...\n";
    return find_eulerian_circuit(*vertices.begin(), aug);
}
