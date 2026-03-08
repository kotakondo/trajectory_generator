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
#include <cmath>
#include <stdexcept>

namespace trajectory_generator {

TrajectoryGeneratorNode::TrajectoryGeneratorNode(const rclcpp::NodeOptions& options)
    : Node("trajectory_generator_node", options)
{
    // Build odom frame ID from namespace (e.g. "/RR03" -> "RR03/odom")
    {
        std::string ns = this->get_namespace();
        if (!ns.empty() && ns.front() == '/') ns = ns.substr(1);
        odom_frame_id_ = ns.empty() ? "odom" : ns + "/odom";
    }

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
    this->declare_parameter<double>("v_fig8", 0.8);
    this->declare_parameter<int>("fig8_laps", 20);

    // Line parameters
    this->declare_parameter<double>("Ax", 0.0);
    this->declare_parameter<double>("Ay", 0.0);
    this->declare_parameter<double>("Bx", 8.0);
    this->declare_parameter<double>("By", 0.0);
    this->declare_parameter<double>("v_line", 0.8);
    this->declare_parameter<int>("num_laps", 20);
    this->declare_parameter<bool>("smooth_turns", true);

    // Read parameters needed before odom arrives
    traj_type_ = this->get_parameter("traj_type").as_string();
    pub_freq_ = this->get_parameter("pub_freq").as_double();
    dt_ = 1.0 / pub_freq_;

    RCLCPP_INFO(this->get_logger(), "Trajectory type: %s", traj_type_.c_str());
    RCLCPP_INFO(this->get_logger(), "Publication frequency: %.1f Hz (dt=%.4f s)", pub_freq_, dt_);

    // Subscribe to odom — trajectory generation deferred until first message
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10,
        std::bind(&TrajectoryGeneratorNode::odomCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Waiting for first odom message to initialize trajectory...");

    // Warning timer: log if no odom arrives within 10s
    odom_warn_timer_ = this->create_wall_timer(
        std::chrono::seconds(10),
        [this]() {
            if (!trajectory_ready_) {
                RCLCPP_WARN(this->get_logger(),
                    "No odom message received after 10s. Is the odom topic remapped correctly?");
            }
            odom_warn_timer_->cancel();
        });
}

void TrajectoryGeneratorNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    if (trajectory_ready_) return;

    // Extract robot pose (x, y, yaw) from first odom message
    double rx = msg->pose.pose.position.x;
    double ry = msg->pose.pose.position.y;

    // Extract yaw from quaternion
    const auto& q = msg->pose.pose.orientation;
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    double ryaw = std::atan2(siny_cosp, cosy_cosp);

    RCLCPP_INFO(this->get_logger(),
        "First odom received: robot at (%.2f, %.2f, yaw=%.2f deg)",
        rx, ry, ryaw * 180.0 / M_PI);

    // Reset odom subscription — no longer needed
    odom_sub_.reset();
    if (odom_warn_timer_) odom_warn_timer_->cancel();

    initializeTrajectory(rx, ry, ryaw);
}

void TrajectoryGeneratorNode::initializeTrajectory(double rx, double ry, double ryaw)
{
    // Instantiate the appropriate trajectory subclass
    if (traj_type_ == "Circle") {
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

        traj_ = std::make_unique<Circle>(r, cx, cy, v_goals, t_traj, accel, dt_);

        RCLCPP_INFO(this->get_logger(), "Circle: r=%.2f, center=(%.2f, %.2f), accel=%.2f",
                    r, cx, cy, accel);
    }
    else if (traj_type_ == "Figure8") {
        double amp_x = this->get_parameter("amplitude_x").as_double();
        double amp_y = this->get_parameter("amplitude_y").as_double();
        double v_fig8 = this->get_parameter("v_fig8").as_double();
        int fig8_laps = this->get_parameter("fig8_laps").as_int();

        traj_ = std::make_unique<Figure8>(amp_x, amp_y, v_fig8, fig8_laps, dt_);

        // Generate trajectory waypoints
        std::vector<geometry_msgs::msg::PoseStamped> waypoints;
        traj_->generateTraj(waypoints);
        RCLCPP_INFO(this->get_logger(), "Figure8: amp_x=%.2f, amp_y=%.2f, v=%.2f, laps=%d, waypoints=%zu",
                    amp_x, amp_y, v_fig8, fig8_laps, waypoints.size());

        applyPoseOffset(waypoints, rx, ry, ryaw);
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
    else if (traj_type_ == "Line") {
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

        bool smooth_turns = this->get_parameter("smooth_turns").as_bool();
        traj_ = std::make_unique<Line>(Ax, Ay, Bx, By, v_line, num_laps, dt_, smooth_turns);

        // Generate trajectory waypoints
        std::vector<geometry_msgs::msg::PoseStamped> waypoints;
        traj_->generateTraj(waypoints);
        RCLCPP_INFO(this->get_logger(), "Line: A=(%.2f, %.2f), B=(%.2f, %.2f), v=%.2f, num_laps=%d, waypoints=%zu",
                    Ax, Ay, Bx, By, v_line, num_laps, waypoints.size());

        applyPoseOffset(waypoints, rx, ry, ryaw);
        buildPath(waypoints);

        // Visualization: show only the nominal A–B line (offset to match)
        // Rotate and translate the A–B endpoints the same way as waypoints
        double orig_yaw = std::atan2(By - Ay, Bx - Ax);
        double dyaw = ryaw - orig_yaw;
        double cos_d = std::cos(dyaw);
        double sin_d = std::sin(dyaw);

        // Rotate A and B around original A, then translate to robot position
        // A stays at robot position after transform
        double oAx = rx;
        double oAy = ry;
        double dBx = Bx - Ax;
        double dBy = By - Ay;
        double oBx = rx + cos_d * dBx - sin_d * dBy;
        double oBy = ry + sin_d * dBx + cos_d * dBy;

        marker_msg_.markers.clear();
        visualization_msgs::msg::Marker line_strip;
        line_strip.header.frame_id = odom_frame_id_;
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
        pa.x = oAx; pa.y = oAy; pa.z = 0.0;
        pb.x = oBx; pb.y = oBy; pb.z = 0.0;
        line_strip.points.push_back(pa);
        line_strip.points.push_back(pb);
        marker_msg_.markers.push_back(line_strip);
    }
    else {
        RCLCPP_ERROR(this->get_logger(), "Unknown traj_type: '%s'. Must be 'Circle', 'Figure8', or 'Line'.",
                     traj_type_.c_str());
        throw std::runtime_error("Invalid traj_type parameter");
    }

    if (path_msg_.poses.empty()) {
        // Generate trajectory waypoints (Circle path — Line/Figure8 do it above)
        std::vector<geometry_msgs::msg::PoseStamped> waypoints;
        traj_->generateTraj(waypoints);
        RCLCPP_INFO(this->get_logger(), "Generated %zu waypoints", waypoints.size());

        applyPoseOffset(waypoints, rx, ry, ryaw);
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

    trajectory_ready_ = true;
    RCLCPP_INFO(this->get_logger(), "Trajectory generator node ready. Publishing at 1 Hz.");
}

void TrajectoryGeneratorNode::applyPoseOffset(
    std::vector<geometry_msgs::msg::PoseStamped>& waypoints,
    double rx, double ry, double ryaw)
{
    if (waypoints.empty()) return;

    // Get the first waypoint's position and yaw
    const auto& first = waypoints.front();
    double wp0_x = first.pose.position.x;
    double wp0_y = first.pose.position.y;
    double wp0_yaw = std::atan2(
        2.0 * (first.pose.orientation.w * first.pose.orientation.z +
               first.pose.orientation.x * first.pose.orientation.y),
        1.0 - 2.0 * (first.pose.orientation.y * first.pose.orientation.y +
                      first.pose.orientation.z * first.pose.orientation.z));

    // Rotation angle: align first waypoint's heading to robot's heading
    double dyaw = ryaw - wp0_yaw;
    double cos_d = std::cos(dyaw);
    double sin_d = std::sin(dyaw);

    RCLCPP_INFO(this->get_logger(),
        "Applying pose offset: wp0=(%.2f, %.2f, %.1f deg) -> robot=(%.2f, %.2f, %.1f deg), dyaw=%.1f deg",
        wp0_x, wp0_y, wp0_yaw * 180.0 / M_PI,
        rx, ry, ryaw * 180.0 / M_PI,
        dyaw * 180.0 / M_PI);

    for (auto& ps : waypoints) {
        // Translate to origin (relative to first waypoint)
        double dx = ps.pose.position.x - wp0_x;
        double dy = ps.pose.position.y - wp0_y;

        // Rotate around origin, then translate to robot position
        ps.pose.position.x = rx + cos_d * dx - sin_d * dy;
        ps.pose.position.y = ry + sin_d * dx + cos_d * dy;

        // Rotate the yaw
        double wp_yaw = std::atan2(
            2.0 * (ps.pose.orientation.w * ps.pose.orientation.z +
                   ps.pose.orientation.x * ps.pose.orientation.y),
            1.0 - 2.0 * (ps.pose.orientation.y * ps.pose.orientation.y +
                          ps.pose.orientation.z * ps.pose.orientation.z));
        double new_yaw = wp_yaw + dyaw;
        ps.pose.orientation.x = 0.0;
        ps.pose.orientation.y = 0.0;
        ps.pose.orientation.z = std::sin(new_yaw / 2.0);
        ps.pose.orientation.w = std::cos(new_yaw / 2.0);
    }
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
    path_msg_.header.frame_id = odom_frame_id_;
    path_msg_.poses = waypoints;
}

void TrajectoryGeneratorNode::buildMarkers(
    const std::vector<geometry_msgs::msg::PoseStamped>& waypoints)
{
    marker_msg_.markers.clear();

    visualization_msgs::msg::Marker line_strip;
    line_strip.header.frame_id = odom_frame_id_;
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
