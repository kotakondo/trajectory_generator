/**
 * @file Figure8.cpp
 * @brief Figure-8 trajectory generation
 * @date 2026-02-27
 */

#include "trajectory_generator/trajectories/Figure8.hpp"
#include <cmath>

namespace trajectory_generator {

void Figure8::generateTraj(std::vector<geometry_msgs::msg::PoseStamped>& waypoints)
{
    waypoints.clear();
    waypoints.reserve(num_points_);

    for (int i = 0; i < num_points_; ++i) {
        double t = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(num_points_);

        // Figure-8 parametric equations:
        //   x = amp_x * sin(t)
        //   y = amp_y * sin(t) * cos(t)
        double x = amp_x_ * std::sin(t);
        double y = amp_y_ * std::sin(t) * std::cos(t);

        // Tangent direction for yaw:
        //   dx/dt = amp_x * cos(t)
        //   dy/dt = amp_y * (cos^2(t) - sin^2(t)) = amp_y * cos(2t)
        double dx_dt = amp_x_ * std::cos(t);
        double dy_dt = amp_y_ * std::cos(2.0 * t);
        double yaw = std::atan2(dy_dt, dx_dt);

        waypoints.push_back(createPoseStamped(x, y, yaw));
    }
}

bool Figure8::trajectoryInsideBounds(double xmin, double xmax,
                                     double ymin, double ymax)
{
    // The figure-8 spans [-amp_x, +amp_x] in X and [-amp_y/2, +amp_y/2] in Y
    return isPointInsideBounds(xmin, xmax, ymin, ymax, -amp_x_, -amp_y_ / 2.0) &&
           isPointInsideBounds(xmin, xmax, ymin, ymax,  amp_x_,  amp_y_ / 2.0);
}

} // namespace trajectory_generator
