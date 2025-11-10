#include <octomap/octomap.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <string>
#include <stdexcept>
#include <cmath>
#include <unordered_set>
#include <fstream> // Added for file writing
#include <random> // For random shuffling
#include <algorithm> // For std::min
#include <sstream> // For formatting text labels

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

// Helper struct for the verification plot
struct VerificationPoint {
    Point3d metric_point;
    Point pixel_point;
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
    tree.getMetricMin(min_x, min_y, min_z); // return coordinates in meters
    tree.getMetricMax(max_x, max_y, max_z); // return coordinates in meters

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


    int min_gx_orig = std::numeric_limits<int>::max(); // Store original min_gx before padding
    int max_gx_orig = std::numeric_limits<int>::min(); // Store original max_gx before padding
    int min_gy_orig = std::numeric_limits<int>::max(); // Store original min_gy before padding
    int max_gy_orig = std::numeric_limits<int>::min(); // Store original max_gy before padding

    for (const auto& pt : occupied_points) {
        min_gx_orig = std::min(min_gx_orig, pt.x);
        max_gx_orig = std::max(max_gx_orig, pt.x);
        min_gy_orig = std::min(min_gy_orig, pt.y);
        max_gy_orig = std::max(max_gy_orig, pt.y);
    }
    
    int min_gx_padded = min_gx_orig;
    int max_gx_padded = max_gx_orig;
    int min_gy_padded = min_gy_orig;
    int max_gy_padded = max_gy_orig;


    // Add a small buffer to avoid drawing at the very edge
    const int PADDING = 10;
    const int CELL_SIZE = 5; 
    
    // Adjust min/max for padding
    min_gx_padded -= PADDING;
    min_gy_padded -= PADDING;
    max_gx_padded += PADDING;
    max_gy_padded += PADDING;

    int img_width = (max_gx_padded - min_gx_padded + 1) * CELL_SIZE;
    int img_height = (max_gy_padded - min_gy_padded + 1) * CELL_SIZE;

    Mat map_image_color = Mat::zeros(img_height, img_width, CV_8UC3);
    Mat map_image_binary = Mat::zeros(img_height, img_width, CV_8U);

    for (const auto& pt : occupied_points) {
        // Offset points by new min_gx/min_gy
        int px = (pt.x - min_gx_padded) * CELL_SIZE;
        int py = (pt.y - min_gy_padded) * CELL_SIZE;
        
        rectangle(map_image_color, Point(px, py), Point(px + CELL_SIZE, py + CELL_SIZE), Scalar(0, 0, 255), FILLED);
        rectangle(map_image_binary, Point(px, py), Point(px + CELL_SIZE, py + CELL_SIZE), Scalar(255), FILLED);
    }

    cout << "\n--- Finding Building Outlines ---" << endl;
    
    Mat labels;
    int num_labels = connectedComponents(map_image_binary, labels, 8); // 8-way connectivity is better

    cout << "Found " << num_labels - 1 << " distinct obstacles (clusters)." << endl;

    vector<vector<Point>> waypoints_per_building; 


    for (int i = 1; i < num_labels; ++i) {
        Mat mask = (labels == i);
        vector<Point> cluster_pixels;
        findNonZero(mask, cluster_pixels);
        if (cluster_pixels.size() < 20) {
            continue; // Skip small noise
        }

        // 1. DILATE (as you requested)
        // This makes the obstacle boundary grow outwards
        Mat dilated_mask;
        // The kernel size determines how much to dilate. 
        // A 5x5 kernel is a good start since your CELL_SIZE is 5.
        Mat element = getStructuringElement(MORPH_RECT, Size(51, 51)); 
        dilate(mask, dilated_mask, element);

        // 2. FIND CONTOURS
        // Find the precise outline of the dilated shape
        vector<vector<Point>> contours;
        findContours(dilated_mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        // 3. APPROXIMATE & DRAW
        // Simplify the contour to a polygon and draw it
        for (size_t j = 0; j < contours.size(); j++) {
            vector<Point> approx_poly;
            
            // Adjust this epsilon value to get more or less detail
            // Smaller value = more detail, Larger value = less detail
            double epsilon = 0.007* arcLength(contours[j], true); 
            approxPolyDP(contours[j], approx_poly, epsilon, true);

            waypoints_per_building.push_back(approx_poly); // Save the simplified polygon

            // Draw the new, accurate polygon
            vector<vector<Point>> poly_to_draw = {approx_poly};
            polylines(map_image_color, poly_to_draw, true, Scalar(0, 255, 255), 2);
        }
    }

    // --- NEW SECTION: How to access and SAVE the polygon points as JSON ---
    cout << "\n--- Accessing Polygon Vertices (Your Graph Nodes) ---" << endl;
    
    // Open a file for writing
    ofstream waypoints_file("waypoints.json");
    if (!waypoints_file.is_open()) {
        cerr << "Error: Could not open waypoints.json for writing!" << endl;
    } else {
        cout << "Saving metric waypoints to waypoints.json" << endl;
        // Write the opening of the JSON object and the buildings array
        waypoints_file << "{\n  \"buildings\": [\n";
    }

    // Store the metric waypoints temporarily for drawing later
    vector<Point3d> all_metric_waypoints; 

    for (size_t i = 0; i < waypoints_per_building.size(); ++i) {
        cout << "Building Outline #" << i << " (Polygon) has " 
             << waypoints_per_building[i].size() << " vertices:" << endl;
        
        if (waypoints_file.is_open()) {
            // Start a new JSON object for this building
            waypoints_file << "    {\n";
            waypoints_file << "      \"id\": " << i << ",\n";
            waypoints_file << "      \"waypoints\": [\n";
        }
        
        const auto& polygon_vertices = waypoints_per_building[i];

        for (size_t j = 0; j < polygon_vertices.size(); ++j) {
            const Point& vertex = polygon_vertices[j]; // This is (px, py)

            // --- Convert pixel coordinate back to metric coordinate ---
            double gx = (double)(vertex.x) / CELL_SIZE + min_gx_padded;
            double gy = (double)(vertex.y) / CELL_SIZE + min_gy_padded;

            double x_meters = gx * resolution;
            double y_meters = gy * resolution;

            // The drone waypoint is (x_meters, y_meters, z_target)
            cout <<"( "<<x_meters<<", "<< y_meters << ", " << z_target << " )"<< endl;

            // Store for drawing
            all_metric_waypoints.emplace_back(x_meters, y_meters, z_target);

            // Write to file as a JSON object
            if (waypoints_file.is_open()) {
                waypoints_file << "        {\"x\": " << x_meters 
                               << ", \"y\": " << y_meters 
                               << ", \"z\": " << z_target << "}";
                // Add a comma if it's not the last waypoint
                if (j < polygon_vertices.size() - 1) {
                    waypoints_file << ",";
                }
                waypoints_file << "\n";
            }
        }
        
        // Close the waypoints array and the building object
        if (waypoints_file.is_open()) {
            waypoints_file << "      ]\n"; // Close waypoints array
            waypoints_file << "    }";     // Close building object
            // Add a comma if it's not the last building
            if (i < waypoints_per_building.size() - 1) {
                waypoints_file << ",";
            }
            waypoints_file << "\n";
        }
    }
    cout << "------------------------------------------" << endl;
    
    // Close the file
    if (waypoints_file.is_open()) {
        waypoints_file << "  ]\n}\n"; // Close buildings array and root object
        waypoints_file.close();
    }
    // --- END OF JSON SAVING SECTION ---

    // --- NEW SECTION: VERIFICATION PLOT ---
    cout << "\n--- Plotting Metric Waypoints for Verification ---" << endl;
    Mat verification_image = map_image_color.clone(); // Clone the colored map to draw on it
    
    vector<VerificationPoint> verification_points; // Store points for labeling

    for (const auto& pt_metric : all_metric_waypoints) {
        // Convert metric point back to image pixel coordinates
        // 1. Meters -> Grid
        double gx_from_metric = pt_metric.x / resolution;
        double gy_from_metric = pt_metric.y / resolution;

        // 2. Grid -> Pixel (using the same min_gx_padded/min_gy_padded and CELL_SIZE)
        int px_verify = static_cast<int>(round((gx_from_metric - min_gx_padded) * CELL_SIZE));
        int py_verify = static_cast<int>(round((gy_from_metric - min_gy_padded) * CELL_SIZE));
        
        // Draw a small green circle at the waypoint location
        circle(verification_image, Point(px_verify, py_verify), 3, Scalar(0, 255, 0), FILLED); // Green circle, radius 3
    
        // Store this point for potential labeling
        verification_points.push_back({pt_metric, Point(px_verify, py_verify)});
    }
    
    // --- Add random labels ---
    // Shuffle the verification points
    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(verification_points.begin(), verification_points.end(), g);

    // Determine how many points to label (up to 10)
    int num_to_label = std::min(10, (int)verification_points.size());
    cout << "Labeling " << num_to_label << " random waypoints..." << endl;

    for (int i = 0; i < num_to_label; ++i) {
        const auto& vp = verification_points[i];
        
        // Format the text label
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(2) // 2 decimal places
           << "(" << vp.metric_point.x << ", " << vp.metric_point.y << ")";
        string label = ss.str();
        
        // Calculate text position (slightly offset from the point)
        Point text_pos = vp.pixel_point + Point(5, -5); // Offset 5px right, 5px up

        // Draw the text
        putText(verification_image,
                label,
                text_pos,
                FONT_HERSHEY_SIMPLEX, // Font type
                0.4, // Font scale
                Scalar(255, 255, 255), // Color (white)
                1, // Thickness
                LINE_AA); // Anti-aliased
    }

    cout << "Displaying map. Red=Obstacle, Yellow=Building Outline, Green=Converted Waypoints, White=Random Labels" << endl;
    imwrite("map_visualization_contours_with_waypoints.png", verification_image);
    // --- END OF VERIFICATION PLOT SECTION ---

    cout << "Original map visualization saved to map_visualization_contours.png" << endl;
    cout << "Verification map visualization saved to map_visualization_contours_with_waypoints.png" << endl;
    cout << "Metric waypoints saved to waypoints.json" << endl;


    return 0;
}