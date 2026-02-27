/**
 * @file Circle.cpp
 * @brief Circular trajectory generation (ROS2 port)
 * @author Aleix Paris (original), ported to ROS2
 * @date 2020-02-18 (original), 2026-02-27 (ROS2 port)
 */

#include "trajectory_generator/trajectories/Circle.hpp"
#include <cmath>

namespace trajectory_generator {

void Circle::generateTraj(std::vector<geometry_msgs::msg::PoseStamped>& waypoints)
{
    waypoints.clear();

    // Initial position on the circle at theta=0
    double theta = 0.0;
    double v = 0.0;

    // Position: (cx + r*cos(theta), cy + r*sin(theta), 0)
    // Yaw: theta + pi/2 (tangent to circle)
    double yaw = theta + M_PI_2;
    waypoints.push_back(createPoseStamped(cx_ + r_ * std::cos(theta),
                                           cy_ + r_ * std::sin(theta),
                                           yaw));

    // For each velocity goal: accelerate, hold constant velocity, then move to next
    for (size_t i = 0; i < v_goals_.size(); ++i) {
        double v_goal = v_goals_[i];

        // Accelerate to the goal velocity
        while (v < v_goal) {
            v = std::min(v + accel_ * dt_, v_goal);
            double omega = v / r_;
            theta += omega * dt_;
            yaw = theta + M_PI_2;

            waypoints.push_back(createPoseStamped(cx_ + r_ * std::cos(theta),
                                                   cy_ + r_ * std::sin(theta),
                                                   yaw));
        }

        // Hold constant velocity for t_traj_ seconds
        double elapsed = 0.0;
        while (elapsed < t_traj_) {
            double omega = v / r_;
            theta += omega * dt_;
            yaw = theta + M_PI_2;

            waypoints.push_back(createPoseStamped(cx_ + r_ * std::cos(theta),
                                                   cy_ + r_ * std::sin(theta),
                                                   yaw));
            elapsed += dt_;
        }
    }

    // Decelerate to zero
    while (v > 0.0) {
        v = std::max(v - accel_ * dt_, 0.0);
        double omega = v / r_;
        theta += omega * dt_;
        yaw = theta + M_PI_2;

        waypoints.push_back(createPoseStamped(cx_ + r_ * std::cos(theta),
                                               cy_ + r_ * std::sin(theta),
                                               yaw));
    }
}

bool Circle::trajectoryInsideBounds(double xmin, double xmax,
                                    double ymin, double ymax)
{
    // Check that the entire circle fits within the bounds
    return isPointInsideBounds(xmin, xmax, ymin, ymax, cx_ - r_, cy_ - r_) &&
           isPointInsideBounds(xmin, xmax, ymin, ymax, cx_ + r_, cy_ + r_);
}

} // namespace trajectory_generator
