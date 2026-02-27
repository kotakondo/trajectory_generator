/**
 * @file Figure8.hpp
 * @brief Figure-8 trajectory class
 * @date 2026-02-27
 */

#pragma once

#include "trajectory_generator/trajectories/Trajectory.hpp"

namespace trajectory_generator {

/**
 * Class that represents a figure-8 (lemniscate) trajectory.
 * Parametric equations:
 *   x = amp_x * sin(t)
 *   y = amp_y * sin(t) * cos(t)
 */
class Figure8 : public Trajectory
{
public:

    Figure8(double amp_x, double amp_y, double velocity, int num_laps, double dt)
        : Trajectory(dt), amp_x_(amp_x), amp_y_(amp_y), velocity_(velocity), num_laps_(num_laps) {}

    ~Figure8() override = default;

    void generateTraj(std::vector<geometry_msgs::msg::PoseStamped>& waypoints) override;

    bool trajectoryInsideBounds(double xmin, double xmax,
                                double ymin, double ymax) override;

private:

    double amp_x_;      // half-width in X (m)
    double amp_y_;      // half-height in Y (m)
    double velocity_;   // desired cruise velocity (m/s)
    int num_laps_;      // number of loops to generate
};

} // namespace trajectory_generator
