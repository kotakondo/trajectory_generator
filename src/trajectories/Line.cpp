/**
 * @file Line.cpp
 * @brief Back-and-forth linear trajectory generation (ROS2 port)
 * @author Aleix Paris (original), ported to ROS2
 * @date 2020-02-19 (original), 2026-02-27 (ROS2 port)
 */

#include "trajectory_generator/trajectories/Line.hpp"
#include <cmath>
#include <algorithm>

namespace trajectory_generator {

void Line::generateTraj(std::vector<geometry_msgs::msg::PoseStamped>& waypoints)
{
    waypoints.clear();

    double dx_AB = Bx_ - Ax_;
    double dy_AB = By_ - Ay_;
    double line_length = std::sqrt(dx_AB * dx_AB + dy_AB * dy_AB);

    double step = v_goal_ * dt_;

    double forward_theta = theta_;
    double reverse_theta = theta_ + M_PI;

    // Smooth U-turn radius at each endpoint.
    // Keeps the robot moving continuously instead of stopping for turn-in-place.
    // The path becomes a racetrack/stadium shape: forward leg, semicircular
    // U-turn, parallel return leg (offset by 2*r), semicircular U-turn back.
    const double turn_radius = 1.0;

    // Perpendicular unit vector (left of A→B direction)
    double perp_x = -std::sin(theta_);
    double perp_y =  std::cos(theta_);

    // Return leg is offset perpendicular to the forward leg by 2*turn_radius
    double offset_x = 2.0 * turn_radius * perp_x;
    double offset_y = 2.0 * turn_radius * perp_y;

    int points_per_leg = std::max(1, static_cast<int>(std::round(line_length / step)));

    // Semicircular U-turn: arc_length = pi * r
    double arc_length = M_PI * turn_radius;
    int points_per_turn = std::max(4, static_cast<int>(std::round(arc_length / step)));

    for (int lap = 0; lap < num_laps_; ++lap)
    {
        // ---- Forward leg: A → B ----
        int fwd_start = (lap == 0) ? 0 : 1;  // avoid duplicate point at lap boundary
        for (int i = fwd_start; i <= points_per_leg; ++i)
        {
            double t = static_cast<double>(i) / points_per_leg;
            double x = Ax_ + t * dx_AB;
            double y = Ay_ + t * dy_AB;
            waypoints.push_back(createPoseStamped(x, y, forward_theta));
        }

        // ---- U-turn at B (counterclockwise semicircle) ----
        {
            double cx = Bx_ + turn_radius * perp_x;
            double cy = By_ + turn_radius * perp_y;
            double start_angle = theta_ - M_PI / 2.0;

            for (int i = 1; i <= points_per_turn; ++i)
            {
                double t = static_cast<double>(i) / points_per_turn;
                double angle = start_angle + t * M_PI;
                double x = cx + turn_radius * std::cos(angle);
                double y = cy + turn_radius * std::sin(angle);
                double heading = angle + M_PI / 2.0;
                waypoints.push_back(createPoseStamped(x, y, heading));
            }
        }

        // ---- Return leg: B_offset → A_offset ----
        for (int i = 1; i <= points_per_leg; ++i)
        {
            double t = static_cast<double>(i) / points_per_leg;
            double x = Bx_ + offset_x - t * dx_AB;
            double y = By_ + offset_y - t * dy_AB;
            waypoints.push_back(createPoseStamped(x, y, reverse_theta));
        }

        // ---- U-turn at A (counterclockwise semicircle) ----
        {
            double cx = Ax_ + turn_radius * perp_x;
            double cy = Ay_ + turn_radius * perp_y;
            double start_angle = theta_ + M_PI / 2.0;

            for (int i = 1; i <= points_per_turn; ++i)
            {
                double t = static_cast<double>(i) / points_per_turn;
                double angle = start_angle + t * M_PI;
                double x = cx + turn_radius * std::cos(angle);
                double y = cy + turn_radius * std::sin(angle);
                double heading = angle + M_PI / 2.0;
                waypoints.push_back(createPoseStamped(x, y, heading));
            }
        }
    }
}

bool Line::trajectoryInsideBounds(double xmin, double xmax,
                                  double ymin, double ymax)
{
    const double turn_radius = 1.0;
    double perp_x = -std::sin(theta_);
    double perp_y =  std::cos(theta_);
    double off_x = 2.0 * turn_radius * perp_x;
    double off_y = 2.0 * turn_radius * perp_y;

    return isPointInsideBounds(xmin, xmax, ymin, ymax, Ax_, Ay_) &&
           isPointInsideBounds(xmin, xmax, ymin, ymax, Bx_, By_) &&
           isPointInsideBounds(xmin, xmax, ymin, ymax, Ax_ + off_x, Ay_ + off_y) &&
           isPointInsideBounds(xmin, xmax, ymin, ymax, Bx_ + off_x, By_ + off_y);
}

} // namespace trajectory_generator
