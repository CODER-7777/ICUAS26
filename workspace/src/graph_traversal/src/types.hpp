#pragma once

#include <rclcpp/rclcpp.hpp>
#include <cstdint>
#include <functional>

struct idx { int i, j; };

struct DroneBatteryState {
    float percentage = 100.0f;
    bool has_charged = false;     // Can only charge once
    bool is_charging = false;     // Currently on ground charging
    bool is_going_to_charge = false; // En route to charging area
    bool is_leaving_charger = false; // NEW: Post-charge clearance maneuver
    rclcpp::Time start_leaving_time; // NEW: Timer for clearance
    int assigned_slot = -1;       // -1: None, 0..N: Slot Index
    float start_charge_percentage = 0.0f; // Battery level when charging started
};

// Node for A* Priority Queue
struct AStarNode {
    int idx_flat;
    int f_score;
    bool operator>(const AStarNode& other) const { return f_score > other.f_score; }
};

enum class SwarmState { TAKEOFF, MISSION, RETURN_TO_HOME };

enum class DroneRole { RTH, LANDING, CHAIN_COMPONENT, SEARCH };

struct PoleInfo {
    double cx, cy;        // XY centre (world)
    double radius;        // approx pole half-width
    double z_bottom;      // bottom of pole
    double z_top;         // top of pole
};

struct ViewZone {
    double x, y, z;       // drone target position
    double yaw;           // facing pole centre
    int pole_idx;         // which pole this zone belongs to
    int dir_idx;          // 0..3 cardinal (E,N,W,S)
    int height_idx;       // 0 = low (25%), 1 = high (75%)
    bool visited = false;
};
