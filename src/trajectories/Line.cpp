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

    // Unit vector A → B
    double ux = dx_AB / line_length;
    double uy = dy_AB / line_length;

    double forward_theta = theta_;
    double reverse_theta = theta_ + M_PI;
    if (reverse_theta > M_PI) reverse_theta -= 2.0 * M_PI;

    double step = v_goal_ * dt_;

    // Extend each leg past the nominal endpoint so the pure pursuit
    // lookahead has real distance before the turn cluster and can't
    // "see" the return leg prematurely.
    const double run_out = 0.3;  // meters past each endpoint (matches PP crossing threshold)
    double ext_length = line_length + 2.0 * run_out;

    // Extended endpoints
    double ext_Ax = Ax_ - run_out * ux;
    double ext_Ay = Ay_ - run_out * uy;
    double ext_Bx = Bx_ + run_out * ux;
    double ext_By = By_ + run_out * uy;
    double ext_dx = ext_Bx - ext_Ax;
    double ext_dy = ext_By - ext_Ay;

    int points_per_leg = std::max(1, static_cast<int>(std::round(ext_length / step)));

    // Turn clusters: a small group of waypoints at the extended endpoint
    // with reversed heading.  The pure pursuit controller's turn-in-place
    // logic (heading error > 60°) handles the 180° rotation naturally.
    const int turn_cluster = 5;

    // Each "lap" is one round trip: ext_A → ext_B → ext_A
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

        // Turn cluster at ext_B (heading reversed)
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

        // Turn cluster at ext_A (heading forward) — except on last lap
        if (lap < num_laps_ - 1)
        {
            for (int i = 0; i < turn_cluster; ++i)
            {
                waypoints.push_back(createPoseStamped(ext_Ax, ext_Ay, forward_theta));
            }
        }
    }
}

bool Line::trajectoryInsideBounds(double xmin, double xmax,
                                  double ymin, double ymax)
{
    return isPointInsideBounds(xmin, xmax, ymin, ymax, Ax_, Ay_) &&
           isPointInsideBounds(xmin, xmax, ymin, ymax, Bx_, By_);
}

} // namespace trajectory_generator
