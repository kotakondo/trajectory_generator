/**
 * @file trajectory_generator_node.cpp
 * @brief TrajectoryGenerator ROS2 node implementation
 * @date 2026-02-27
 */

#include "trajectory_generator/trajectory_generator_node.hpp"
#include "trajectory_generator/trajectories/Circle.hpp"
#include "trajectory_generator/trajectories/Figure8.hpp"
#include "trajectory_generator/trajectories/Line.hpp"

#include <chrono>
#include <stdexcept>

namespace trajectory_generator {

TrajectoryGeneratorNode::TrajectoryGeneratorNode(const rclcpp::NodeOptions& options)
    : Node("trajectory_generator_node", options)
{
    // Declare parameters with defaults
    this->declare_parameter<std::string>("traj_type", "Circle");
    this->declare_parameter<double>("pub_freq", 10.0);

    // Circle parameters (scaled for ground robot)
    this->declare_parameter<double>("r", 3.0);
    this->declare_parameter<double>("center_x", 0.0);
    this->declare_parameter<double>("center_y", 0.0);
    this->declare_parameter<std::vector<double>>("v_goals", {0.3, 0.5, 0.8, 1.0});
    this->declare_parameter<double>("t_traj", 20.0);
    this->declare_parameter<double>("circle_accel", 0.3);

    // Figure-8 parameters
    this->declare_parameter<double>("amplitude_x", 5.0);
    this->declare_parameter<double>("amplitude_y", 3.0);
    this->declare_parameter<int>("num_points", 200);

    // Line parameters
    this->declare_parameter<double>("Ax", 0.0);
    this->declare_parameter<double>("Ay", 0.0);
    this->declare_parameter<double>("Bx", 8.0);
    this->declare_parameter<double>("By", 0.0);
    this->declare_parameter<double>("v_line", 0.8);
    this->declare_parameter<int>("num_laps", 20);

    // Read parameters
    std::string traj_type = this->get_parameter("traj_type").as_string();
    double pub_freq = this->get_parameter("pub_freq").as_double();
    double dt = 1.0 / pub_freq;

    RCLCPP_INFO(this->get_logger(), "Trajectory type: %s", traj_type.c_str());
    RCLCPP_INFO(this->get_logger(), "Publication frequency: %.1f Hz (dt=%.4f s)", pub_freq, dt);

    // Instantiate the appropriate trajectory subclass
    if (traj_type == "Circle") {
        double r = this->get_parameter("r").as_double();
        double cx = this->get_parameter("center_x").as_double();
        double cy = this->get_parameter("center_y").as_double();
        std::vector<double> v_goals = this->get_parameter("v_goals").as_double_array();
        double t_traj = this->get_parameter("t_traj").as_double();
        double accel = this->get_parameter("circle_accel").as_double();

        // Validate
        for (double vel : v_goals) {
            if (vel <= 0.0) {
                RCLCPP_ERROR(this->get_logger(), "All velocities must be > 0");
                throw std::runtime_error("Invalid velocity goal");
            }
        }
        if (accel <= 0.0) {
            RCLCPP_ERROR(this->get_logger(), "circle_accel must be > 0");
            throw std::runtime_error("Invalid acceleration");
        }

        traj_ = std::make_unique<Circle>(r, cx, cy, v_goals, t_traj, accel, dt);

        RCLCPP_INFO(this->get_logger(), "Circle: r=%.2f, center=(%.2f, %.2f), accel=%.2f",
                    r, cx, cy, accel);
    }
    else if (traj_type == "Figure8") {
        double amp_x = this->get_parameter("amplitude_x").as_double();
        double amp_y = this->get_parameter("amplitude_y").as_double();
        int num_points = this->get_parameter("num_points").as_int();

        traj_ = std::make_unique<Figure8>(amp_x, amp_y, num_points, dt);

        // Generate trajectory waypoints
        std::vector<geometry_msgs::msg::PoseStamped> waypoints;
        traj_->generateTraj(waypoints);
        RCLCPP_INFO(this->get_logger(), "Figure8: amp_x=%.2f, amp_y=%.2f, num_points=%d, waypoints=%zu",
                    amp_x, amp_y, num_points, waypoints.size());

        buildPath(waypoints);

        // Build markers — close the loop by appending first point
        buildMarkers(waypoints);
        if (!marker_msg_.markers.empty() && !waypoints.empty()) {
            geometry_msgs::msg::Point pt;
            pt.x = waypoints.front().pose.position.x;
            pt.y = waypoints.front().pose.position.y;
            pt.z = waypoints.front().pose.position.z;
            marker_msg_.markers[0].points.push_back(pt);
        }
    }
    else if (traj_type == "Line") {
        double Ax = this->get_parameter("Ax").as_double();
        double Ay = this->get_parameter("Ay").as_double();
        double Bx = this->get_parameter("Bx").as_double();
        double By = this->get_parameter("By").as_double();
        double v_line = this->get_parameter("v_line").as_double();
        int num_laps = this->get_parameter("num_laps").as_int();

        // Validate
        if (v_line <= 0.0) {
            RCLCPP_ERROR(this->get_logger(), "v_line must be > 0");
            throw std::runtime_error("Invalid velocity");
        }
        if (num_laps <= 0) {
            RCLCPP_ERROR(this->get_logger(), "num_laps must be > 0");
            throw std::runtime_error("Invalid num_laps");
        }

        traj_ = std::make_unique<Line>(Ax, Ay, Bx, By, v_line, num_laps, dt);

        // Generate trajectory waypoints
        std::vector<geometry_msgs::msg::PoseStamped> waypoints;
        traj_->generateTraj(waypoints);
        RCLCPP_INFO(this->get_logger(), "Line: A=(%.2f, %.2f), B=(%.2f, %.2f), v=%.2f, num_laps=%d, waypoints=%zu",
                    Ax, Ay, Bx, By, v_line, num_laps, waypoints.size());

        buildPath(waypoints);

        // Visualization: show only the nominal A–B line (not the run-out extensions)
        marker_msg_.markers.clear();
        visualization_msgs::msg::Marker line_strip;
        line_strip.header.frame_id = "odom";
        line_strip.header.stamp = this->now();
        line_strip.ns = "trajectory";
        line_strip.id = 0;
        line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line_strip.action = visualization_msgs::msg::Marker::ADD;
        line_strip.scale.x = 0.05;
        line_strip.color.g = 1.0f;
        line_strip.color.a = 1.0f;
        line_strip.pose.orientation.w = 1.0;
        geometry_msgs::msg::Point pa, pb;
        pa.x = Ax; pa.y = Ay; pa.z = 0.0;
        pb.x = Bx; pb.y = By; pb.z = 0.0;
        line_strip.points.push_back(pa);
        line_strip.points.push_back(pb);
        marker_msg_.markers.push_back(line_strip);
    }
    else {
        RCLCPP_ERROR(this->get_logger(), "Unknown traj_type: '%s'. Must be 'Circle', 'Figure8', or 'Line'.",
                     traj_type.c_str());
        throw std::runtime_error("Invalid traj_type parameter");
    }

    if (path_msg_.poses.empty()) {
        // Generate trajectory waypoints (Circle path — Line does it above)
        std::vector<geometry_msgs::msg::PoseStamped> waypoints;
        traj_->generateTraj(waypoints);
        RCLCPP_INFO(this->get_logger(), "Generated %zu waypoints", waypoints.size());

        buildPath(waypoints);
        buildMarkers(waypoints);
    }

    // Create publishers
    // /reference_trajectory: reliable + transient_local (acts as latched)
    rclcpp::QoS latched_qos(10);
    latched_qos.reliable();
    latched_qos.transient_local();

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        "reference_trajectory", latched_qos);

    // trajectory_markers: default QoS
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "trajectory_markers", 10);

    // Create timer at 1 Hz to republish
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&TrajectoryGeneratorNode::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "Trajectory generator node ready. Publishing at 1 Hz.");
}

void TrajectoryGeneratorNode::timerCallback()
{
    // Update timestamp
    path_msg_.header.stamp = this->now();
    for (auto& ps : path_msg_.poses) {
        ps.header.stamp = path_msg_.header.stamp;
    }

    path_pub_->publish(path_msg_);
    marker_pub_->publish(marker_msg_);
}

void TrajectoryGeneratorNode::buildPath(
    const std::vector<geometry_msgs::msg::PoseStamped>& waypoints)
{
    path_msg_.header.frame_id = "odom";
    path_msg_.poses = waypoints;
}

void TrajectoryGeneratorNode::buildMarkers(
    const std::vector<geometry_msgs::msg::PoseStamped>& waypoints)
{
    marker_msg_.markers.clear();

    visualization_msgs::msg::Marker line_strip;
    line_strip.header.frame_id = "odom";
    line_strip.header.stamp = this->now();
    line_strip.ns = "trajectory";
    line_strip.id = 0;
    line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_strip.action = visualization_msgs::msg::Marker::ADD;
    line_strip.scale.x = 0.05;  // line width
    line_strip.color.r = 0.0f;
    line_strip.color.g = 1.0f;
    line_strip.color.b = 0.0f;
    line_strip.color.a = 1.0f;
    line_strip.pose.orientation.w = 1.0;

    for (const auto& ps : waypoints) {
        geometry_msgs::msg::Point pt;
        pt.x = ps.pose.position.x;
        pt.y = ps.pose.position.y;
        pt.z = ps.pose.position.z;
        line_strip.points.push_back(pt);
    }

    marker_msg_.markers.push_back(line_strip);
}

} // namespace trajectory_generator

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<trajectory_generator::TrajectoryGeneratorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
