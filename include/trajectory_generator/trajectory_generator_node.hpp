/**
 * @file trajectory_generator_node.hpp
 * @brief TrajectoryGenerator ROS2 node header
 * @date 2026-02-27
 */

#pragma once

#include <memory>
#include <vector>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "trajectory_generator/trajectories/Trajectory.hpp"

namespace trajectory_generator {

/**
 * Main ROS2 node for trajectory generation.
 * Simplified for ground robot: no flight state machine.
 * On startup, generates a trajectory (Circle or Line) and publishes it
 * as a nav_msgs/Path on /reference_trajectory with reliable+transient_local QoS.
 * Also publishes visualization markers on /trajectory_markers.
 */
class TrajectoryGeneratorNode : public rclcpp::Node
{
public:
    explicit TrajectoryGeneratorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    void timerCallback();

    void buildPath(const std::vector<geometry_msgs::msg::PoseStamped>& waypoints);
    void buildMarkers(const std::vector<geometry_msgs::msg::PoseStamped>& waypoints);

    // Trajectory
    std::unique_ptr<Trajectory> traj_;

    // Pre-computed messages
    nav_msgs::msg::Path path_msg_;
    visualization_msgs::msg::MarkerArray marker_msg_;

    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace trajectory_generator
