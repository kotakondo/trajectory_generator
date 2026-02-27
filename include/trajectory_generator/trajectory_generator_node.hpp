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
#include <nav_msgs/msg/odometry.hpp>
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
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void initializeTrajectory(double rx, double ry, double ryaw);
    void applyPoseOffset(std::vector<geometry_msgs::msg::PoseStamped>& waypoints,
                         double rx, double ry, double ryaw);

    void buildPath(const std::vector<geometry_msgs::msg::PoseStamped>& waypoints);
    void buildMarkers(const std::vector<geometry_msgs::msg::PoseStamped>& waypoints);

    // Trajectory
    std::unique_ptr<Trajectory> traj_;

    // Frame ID derived from namespace (e.g. "RR03/odom")
    std::string odom_frame_id_;

    // Deferred initialization state
    bool trajectory_ready_ = false;
    std::string traj_type_;
    double pub_freq_;
    double dt_;

    // Odom subscription (reset after first message)
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr odom_warn_timer_;

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
