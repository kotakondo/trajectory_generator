/**
 * @file perfect_tracker_node.cpp
 * @brief Minimal "perfect tracker" for a drone: consumes dynus_interfaces/Goal and
 *        instantly places the drone at the commanded state. Publishes PoseStamped,
 *        TF, a drone marker, and an accumulated actual-path trail for RViz.
 *
 * Attitude is derived from the commanded acceleration + yaw using the Hopf
 * fibration (matching dynus fake_sim), so the body visibly tilts into accelerations.
 *
 * @date 2026-05-28
 */
#include <rclcpp/rclcpp.hpp>
#include <dynus_interfaces/msg/goal.hpp>
#include <dynus_interfaces/msg/state.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <cmath>
#include <memory>
#include <vector>

namespace trajectory_generator {

class PerfectTrackerNode : public rclcpp::Node {
public:
    PerfectTrackerNode() : Node("perfect_tracker_node")
    {
        std::vector<double> start = this->declare_parameter<std::vector<double>>(
            "start_pos", std::vector<double>{0.0, 0.0, 0.0});
        if (start.size() != 3) start = {0.0, 0.0, 0.0};
        frame_id_    = this->declare_parameter<std::string>("frame_id", "world");
        child_frame_ = this->declare_parameter<std::string>("child_frame", "base_link");
        const std::string pose_topic = this->declare_parameter<std::string>("pose_topic", "world");
        const double pub_rate = this->declare_parameter<double>("pub_rate", 100.0);
        trail_max_ = static_cast<size_t>(this->declare_parameter<int>("trail_max", 5000));

        state_.header.frame_id = frame_id_;
        state_.pos.x = start[0]; state_.pos.y = start[1]; state_.pos.z = start[2];
        state_.quat.w = 1.0;

        rclcpp::QoS qos(rclcpp::KeepLast(10));
        qos.reliable().durability_volatile();
        pub_pose_   = this->create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic, qos);
        pub_marker_ = this->create_publisher<visualization_msgs::msg::Marker>("drone_marker", qos);
        pub_path_   = this->create_publisher<nav_msgs::msg::Path>("actual_path", qos);

        sub_goal_ = this->create_subscription<dynus_interfaces::msg::Goal>(
            "goal", qos, std::bind(&PerfectTrackerNode::goalCallback, this, std::placeholders::_1));

        br_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        path_.header.frame_id = frame_id_;

        const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / pub_rate));
        timer_ = this->create_wall_timer(period, std::bind(&PerfectTrackerNode::publishAll, this));

        RCLCPP_INFO(this->get_logger(),
                    "Perfect tracker ready. Start [%.2f, %.2f, %.2f], frame '%s', child '%s', %.0f Hz.",
                    start[0], start[1], start[2], frame_id_.c_str(), child_frame_.c_str(), pub_rate);
    }

private:
    void goalCallback(const dynus_interfaces::msg::Goal::SharedPtr g)
    {
        state_.pos = g->p;

        // Hopf-fibration attitude from thrust (a + g*z) and yaw.
        const double tx = g->a.x, ty = g->a.y, tz = g->a.z + 9.81;
        const double n = std::sqrt(tx * tx + ty * ty + tz * tz);
        const double a = tx / n, b = ty / n, c = tz / n;
        const double tmp = 1.0 / std::sqrt(2.0 * (1.0 + c));
        tf2::Quaternion qabc(-b * tmp, a * tmp, 0.0, tmp * (1.0 + c));
        tf2::Quaternion qpsi(0.0, 0.0, std::sin(g->yaw / 2.0), std::cos(g->yaw / 2.0));
        tf2::Quaternion q = qabc * qpsi;
        state_.quat.x = q.x(); state_.quat.y = q.y();
        state_.quat.z = q.z(); state_.quat.w = q.w();

        got_goal_ = true;
    }

    void publishAll()
    {
        const rclcpp::Time now = this->now();

        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = now;
        pose.header.frame_id = frame_id_;
        pose.pose.position.x = state_.pos.x;
        pose.pose.position.y = state_.pos.y;
        pose.pose.position.z = state_.pos.z;
        pose.pose.orientation = state_.quat;
        pub_pose_->publish(pose);

        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = now;
        tf.header.frame_id = frame_id_;
        tf.child_frame_id = child_frame_;
        tf.transform.translation.x = state_.pos.x;
        tf.transform.translation.y = state_.pos.y;
        tf.transform.translation.z = state_.pos.z;
        tf.transform.rotation = state_.quat;
        br_->sendTransform(tf);

        visualization_msgs::msg::Marker m;
        m.header.stamp = now;
        m.header.frame_id = frame_id_;
        m.ns = "drone";
        m.id = 0;
        m.type = visualization_msgs::msg::Marker::SPHERE;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.position.x = state_.pos.x;
        m.pose.position.y = state_.pos.y;
        m.pose.position.z = state_.pos.z;
        m.pose.orientation = state_.quat;
        m.scale.x = 0.45; m.scale.y = 0.45; m.scale.z = 0.12;  // disk-like
        m.color.r = 1.0f; m.color.g = 0.55f; m.color.b = 0.0f; m.color.a = 1.0f;
        pub_marker_->publish(m);

        // Accumulate the actual-path trail (only once we're actively tracking).
        if (got_goal_) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header.stamp = now;
            ps.header.frame_id = frame_id_;
            ps.pose.position.x = state_.pos.x;
            ps.pose.position.y = state_.pos.y;
            ps.pose.position.z = state_.pos.z;
            ps.pose.orientation = state_.quat;
            path_.poses.push_back(ps);
            if (path_.poses.size() > trail_max_)
                path_.poses.erase(path_.poses.begin());
            path_.header.stamp = now;
            pub_path_->publish(path_);
        }
    }

    std::string frame_id_, child_frame_;
    size_t trail_max_{5000};
    bool got_goal_{false};
    dynus_interfaces::msg::State state_;
    nav_msgs::msg::Path path_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
    rclcpp::Subscription<dynus_interfaces::msg::Goal>::SharedPtr sub_goal_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> br_;
    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace trajectory_generator

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<trajectory_generator::PerfectTrackerNode>());
    rclcpp::shutdown();
    return 0;
}
