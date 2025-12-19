#include <iostream>
#include <vector>
#include <cmath>
#include <map>
#include <vector>
#include <queue>
#include <algorithm>
#include <limits>
#include <set>
#include <stack>
#include <fstream>
#include <cstdlib>

// 1. Include the JSON library
#include "json.hpp"

// 2. Include all our solver functions
#include "graph_solver.hpp"

// Use the nlohmann::json namespace
using json = nlohmann::json;

int main() {
    
    // === 1. READ AND PARSE THE JSON FILE ===
    
    std::cout << "Reading icuas_waypoints.json..." << std::endl;
    std::ifstream f("icuas_waypoints.json");
    if (!f.is_open()) {
        std::cerr << "Error: Could not open icuas_waypoints.json" << std::endl;
        return 1;
    }
    json data;
    try {
        data = json::parse(f);
    } catch (json::parse_error& e) {
        std::cerr << "Error parsing JSON: " << e.what() << std::endl;
        return 1;
    }

    // === 2. POPULATE OUR C++ DATA STRUCTURES ===
    
    // These are the 3 inputs our functions need
    std::map<int, Point3D> all_vertices;
    std::map<int, int> vertex_to_graph_id;
    std::vector<Edge> intra_graph_edges;
    
    std::set<int> all_vertex_ids;

    // We need to create unique IDs for every waypoint
    int global_vertex_id_counter = 1;

    std::cout << "Loading graph data and generating edges..." << std::endl;

    for (const auto& building : data["buildings"]) {
        int g_id = building["id"];
        
        // This vector stores the new IDs for this building's waypoints
        std::vector<int> current_building_vertex_ids;

        // --- 2.A: Load all vertices for this graph ---
        for (const auto& waypoint : building["waypoints"]) {
            // Assign a new, unique ID
            int new_id = global_vertex_id_counter++;
            
            Point3D p = {
                waypoint["x"],
                waypoint["y"],
                waypoint["z"]
            };
            
            // Store the data
            all_vertices[new_id] = p;
            vertex_to_graph_id[new_id] = g_id;
            all_vertex_ids.insert(new_id);
            current_building_vertex_ids.push_back(new_id);
        }
        
        // --- 2.B: Create edges based on our "closed loop" assumption ---
        if (current_building_vertex_ids.size() < 2) {
            // Can't form an edge with one or zero points
            continue; 
        }

        for (int i = 0; i < current_building_vertex_ids.size(); ++i) {
            int u = current_building_vertex_ids[i];
            
            // This is the magic: (i + 1) % size
            // When i = last_index, (i + 1) % size becomes 0
            // This connects the last vertex back to the first one.
            int v = current_building_vertex_ids[(i + 1) % current_building_vertex_ids.size()];

            // Calculate the edge weight
            double weight = get_distance(all_vertices[u], all_vertices[v]);
            intra_graph_edges.push_back({u, v, weight});
        }
    }
    
    std::cout << "Successfully loaded " << data["buildings"].size() 
              << " graphs, creating " << all_vertices.size() 
              << " unique vertices and " << intra_graph_edges.size() 
              << " intra-graph edges." << std::endl;

    std::cout << "Writing vertices.csv for plotter..." << std::endl;
    std::ofstream vert_file("vertices.csv");
    vert_file << "id,x,y,z\n"; // CSV Header
    for (const auto& pair : all_vertices) {
        vert_file << pair.first << "," 
                  << pair.second.x << ","
                  << pair.second.y << ","
                  << pair.second.z << "\n";
    }
    vert_file.close();          


    // === 3. RUN PHASE 1 (Find Bridges) ===
    
    std::vector<Edge> bridge_edges = find_connecting_bridges(
        all_vertices, 
        vertex_to_graph_id
    );

   // === 4. RUN PHASE 2 (Solve CPP) ===
    
    // Call the new function that returns a path
    std::vector<int> final_tour_path = solve_chinese_postman_path(
        intra_graph_edges, 
        bridge_edges,
        all_vertex_ids
    );

    std::cout << "\n==============================================" << std::endl;
    std::cout << "Final Shortest Tour Path:" << std::endl;
    std::cout << "==============================================" << std::endl;

    std::cout << "Writing final_path.txt for plotter..." << std::endl;
    std::ofstream path_file("final_path.txt");
    for (int v_id : final_tour_path) {
        path_file << v_id << "\n";
    }
    path_file.close();

    // === Write corresponding coordinates to final_path_coordinates.csv ===
    std::cout << "Writing final_path_coordinates.csv..." << std::endl;
    std::ofstream coord_file("final_path_coordinates.csv");
    coord_file << "x,y,z\n";  // CSV header
    for (int v_id : final_tour_path) {
        const Point3D& p = all_vertices.at(v_id);
        coord_file << p.x << "," 
                   << p.y << "," 
                   << p.z << "\n";
    }
    coord_file.close();


    // Print the path
    for (size_t i = 0; i < final_tour_path.size(); ++i) {
        std::cout << final_tour_path[i];
        if (i < final_tour_path.size() - 1) {
            std::cout << " -> ";
        }
    }
    std::cout << std::endl;
    std::cout << "\nPath contains " << final_tour_path.size() << " steps (vertices)." << std::endl;

    return 0;
}