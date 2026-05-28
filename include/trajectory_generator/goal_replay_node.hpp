/**
 * @file goal_replay_node.hpp
 * @brief Replays a pre-generated dynus_interfaces/Goal trajectory from a CSV file.
 * @date 2026-05-28
 */
#ifndef TRAJECTORY_GENERATOR_GOAL_REPLAY_NODE_HPP
#define TRAJECTORY_GENERATOR_GOAL_REPLAY_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <dynus_interfaces/msg/goal.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#include <array>
#include <optional>
#include <string>
#include <vector>

namespace trajectory_generator {

class GoalReplayNode : public rclcpp::Node {
public:
    explicit GoalReplayNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    void loadCsv(const std::string& path);
    void startReplay();
    void tick();
    void onTrigger(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    // Build a quintic (min-jerk) approach from the current measured pose to the
    // first CSV goal. Returns an empty vector if no current pose is known.
    std::vector<dynus_interfaces::msg::Goal> buildApproach() const;

    std::vector<dynus_interfaces::msg::Goal> goals_;       // the CSV trajectory
    std::vector<dynus_interfaces::msg::Goal> playback_;    // approach + CSV, built on start
    std::string frame_id_;
    double dt_{0.01};          // nominal CSV time step (s)
    double rate_scale_{1.0};   // playback speed multiplier (>1 = faster)

    // Approach-planning parameters.
    bool approach_enabled_{true};
    double approach_speed_{1.0};     // cruise speed used to size the approach duration (m/s)
    double approach_min_time_{2.0};  // minimum approach duration (s)
    double takeoff_z_{1.0};          // altitude to climb to in place before transiting (m)

    // Latest measured drone position, if received. (Approach is planned from rest,
    // so only position is needed — velocity is taken as zero.)
    std::optional<std::array<double, 3>> cur_pos_;

    size_t idx_{0};
    bool running_{false};

    rclcpp::TimerBase::SharedPtr start_delay_timer_;

    rclcpp::Publisher<dynus_interfaces::msg::Goal>::SharedPtr goal_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr ref_path_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_srv_;
    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace trajectory_generator

#endif // TRAJECTORY_GENERATOR_GOAL_REPLAY_NODE_HPP
