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

    if (smooth_turns_) {
        // Smooth U-turn mode: racetrack/stadium shape with semicircular arcs
        const double turn_radius = 1.0;

        double perp_x = -std::sin(theta_);
        double perp_y =  std::cos(theta_);

        double offset_x = 2.0 * turn_radius * perp_x;
        double offset_y = 2.0 * turn_radius * perp_y;

        int points_per_leg = std::max(1, static_cast<int>(std::round(line_length / step)));

        double arc_length = M_PI * turn_radius;
        int points_per_turn = std::max(4, static_cast<int>(std::round(arc_length / step)));

        for (int lap = 0; lap < num_laps_; ++lap)
        {
            int fwd_start = (lap == 0) ? 0 : 1;
            for (int i = fwd_start; i <= points_per_leg; ++i)
            {
                double t = static_cast<double>(i) / points_per_leg;
                double x = Ax_ + t * dx_AB;
                double y = Ay_ + t * dy_AB;
                waypoints.push_back(createPoseStamped(x, y, forward_theta));
            }

            // U-turn at B
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

            // Return leg: B_offset → A_offset
            for (int i = 1; i <= points_per_leg; ++i)
            {
                double t = static_cast<double>(i) / points_per_leg;
                double x = Bx_ + offset_x - t * dx_AB;
                double y = By_ + offset_y - t * dy_AB;
                waypoints.push_back(createPoseStamped(x, y, reverse_theta));
            }

            // U-turn at A
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
    } else {
        // Turn-in-place mode: straight A→B→A with turn clusters at endpoints
        double ux = dx_AB / line_length;
        double uy = dy_AB / line_length;

        if (reverse_theta > M_PI) reverse_theta -= 2.0 * M_PI;

        // Extend legs past endpoints so pure pursuit lookahead has room
        const double run_out = 0.3;
        double ext_length = line_length + 2.0 * run_out;

        double ext_Ax = Ax_ - run_out * ux;
        double ext_Ay = Ay_ - run_out * uy;
        double ext_Bx = Bx_ + run_out * ux;
        double ext_By = By_ + run_out * uy;
        double ext_dx = ext_Bx - ext_Ax;
        double ext_dy = ext_By - ext_Ay;

        int points_per_leg = std::max(1, static_cast<int>(std::round(ext_length / step)));

        const int turn_cluster = 5;

        for (int lap = 0; lap < num_laps_; ++lap)
        {
            // Forward leg: ext_A → ext_B
            for (int i = 0; i <= points_per_leg; ++i)
            {
                double t = static_cast<double>(i) / points_per_leg;
                double x = ext_Ax + t * ext_dx;
                double y = ext_Ay + t * ext_dy;
                waypoints.push_back(createPoseStamped(x, y, forward_theta));
            }

            // Turn cluster at ext_B
            for (int i = 0; i < turn_cluster; ++i)
            {
                waypoints.push_back(createPoseStamped(ext_Bx, ext_By, reverse_theta));
            }

            // Return leg: ext_B → ext_A
            for (int i = 0; i <= points_per_leg; ++i)
            {
                double t = static_cast<double>(i) / points_per_leg;
                double x = ext_Bx - t * ext_dx;
                double y = ext_By - t * ext_dy;
                waypoints.push_back(createPoseStamped(x, y, reverse_theta));
            }

            // Turn cluster at ext_A (except last lap)
            if (lap < num_laps_ - 1)
            {
                for (int i = 0; i < turn_cluster; ++i)
                {
                    waypoints.push_back(createPoseStamped(ext_Ax, ext_Ay, forward_theta));
                }
            }
        }
    }
}

bool Line::trajectoryInsideBounds(double xmin, double xmax,
                                  double ymin, double ymax)
{
    if (smooth_turns_) {
        const double turn_radius = 1.0;
        double perp_x = -std::sin(theta_);
        double perp_y =  std::cos(theta_);
        double off_x = 2.0 * turn_radius * perp_x;
        double off_y = 2.0 * turn_radius * perp_y;

        return isPointInsideBounds(xmin, xmax, ymin, ymax, Ax_, Ay_) &&
               isPointInsideBounds(xmin, xmax, ymin, ymax, Bx_, By_) &&
               isPointInsideBounds(xmin, xmax, ymin, ymax, Ax_ + off_x, Ay_ + off_y) &&
               isPointInsideBounds(xmin, xmax, ymin, ymax, Bx_ + off_x, By_ + off_y);
    } else {
        return isPointInsideBounds(xmin, xmax, ymin, ymax, Ax_, Ay_) &&
               isPointInsideBounds(xmin, xmax, ymin, ymax, Bx_, By_);
    }
}

} // namespace trajectory_generator
