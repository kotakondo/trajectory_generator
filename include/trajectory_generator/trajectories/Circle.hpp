/**
 * @file Circle.hpp
 * @brief Circular trajectory class (ROS2 port)
 * @author Aleix Paris (original), ported to ROS2
 * @date 2020-02-18 (original), 2026-02-27 (ROS2 port)
 */

#pragma once

#include "trajectory_generator/trajectories/Trajectory.hpp"

namespace trajectory_generator {

/**
 * Class that represents a circular trajectory.
 * Generates waypoints along a circle with accel/constant-vel/decel phases
 * for each velocity goal.
 */
class Circle : public Trajectory
{
public:

    Circle(double r, double cx, double cy,
           std::vector<double> v_goals, double t_traj, double accel, double dt)
        : Trajectory(dt), r_(r), cx_(cx), cy_(cy),
          v_goals_(std::move(v_goals)), t_traj_(t_traj), accel_(accel) {}

    ~Circle() override = default;

    void generateTraj(std::vector<geometry_msgs::msg::PoseStamped>& waypoints) override;

    bool trajectoryInsideBounds(double xmin, double xmax,
                                double ymin, double ymax) override;

private:

    double r_;          // radius in m
    double cx_, cy_;    // circle center in m
    std::vector<double> v_goals_;  // norm of goal velocities
    double t_traj_;     // time to follow each constant-vel segment, s
    double accel_;      // max acceleration allowed, m/s^2
};

} // namespace trajectory_generator
