#pragma once

#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>

#include <string>

namespace swarm_viz {

inline std_msgs::msg::ColorRGBA makeColor(float r, float g, float b, float a = 1.0f) {
    std_msgs::msg::ColorRGBA c;
    c.r = r; c.g = g; c.b = b; c.a = a;
    return c;
}

inline visualization_msgs::msg::Marker
makeSphere(const std::string& frame, const std::string& ns, int id,
           const geometry_msgs::msg::Point& p, double diameter,
           const std_msgs::msg::ColorRGBA& color,
           const builtin_interfaces::msg::Time& stamp) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame;
    m.header.stamp = stamp;
    m.ns = ns;
    m.id = id;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position = p;
    m.pose.orientation.w = 1.0;
    m.scale.x = m.scale.y = m.scale.z = diameter;
    m.color = color;
    return m;
}

inline visualization_msgs::msg::Marker
makeTextLabel(const std::string& frame, const std::string& ns, int id,
              const geometry_msgs::msg::Point& p, const std::string& text,
              double height, const std_msgs::msg::ColorRGBA& color,
              const builtin_interfaces::msg::Time& stamp) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame;
    m.header.stamp = stamp;
    m.ns = ns;
    m.id = id;
    m.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position = p;
    m.pose.position.z += height * 1.5;
    m.pose.orientation.w = 1.0;
    m.scale.z = height;
    m.color = color;
    m.text = text;
    return m;
}

inline visualization_msgs::msg::Marker
makeLineList(const std::string& frame, const std::string& ns, int id,
             double thickness,
             const builtin_interfaces::msg::Time& stamp) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame;
    m.header.stamp = stamp;
    m.ns = ns;
    m.id = id;
    m.type = visualization_msgs::msg::Marker::LINE_LIST;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.scale.x = thickness;
    m.color.a = 1.0;
    return m;
}

inline visualization_msgs::msg::Marker
makeDeleteAll(const std::string& ns) {
    visualization_msgs::msg::Marker m;
    m.ns = ns;
    m.action = visualization_msgs::msg::Marker::DELETEALL;
    return m;
}

}  // namespace swarm_viz
