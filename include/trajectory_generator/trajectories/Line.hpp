/**
 * @file Line.hpp
 * @brief Back-and-forth linear trajectory class (ROS2 port)
 * @author Aleix Paris (original), ported to ROS2
 * @date 2020-02-19 (original), 2026-02-27 (ROS2 port)
 */

#pragma once

#include "trajectory_generator/trajectories/Trajectory.hpp"

namespace trajectory_generator {

/**
 * Class that represents a back-and-forth linear trajectory between points A and B.
 * Generates num_laps round trips (A→B→A) at constant velocity with turn-in-place
 * clusters at each endpoint.
 */
class Line : public Trajectory
{
public:

    Line(double Ax, double Ay, double Bx, double By,
         double v_goal, int num_laps, double dt)
        : Trajectory(dt), Ax_(Ax), Ay_(Ay), Bx_(Bx), By_(By),
          v_goal_(v_goal), num_laps_(num_laps)
    {
        theta_ = std::atan2(By_ - Ay_, Bx_ - Ax_);
    }

    ~Line() override = default;

    void generateTraj(std::vector<geometry_msgs::msg::PoseStamped>& waypoints) override;

    bool trajectoryInsideBounds(double xmin, double xmax,
                                double ymin, double ymax) override;

private:

    double Ax_, Ay_;    // start point
    double Bx_, By_;    // end point
    double v_goal_;     // cruise velocity, m/s
    int num_laps_;      // number of round trips (A→B→A = 1 lap)
    double theta_;      // angle from A to B
};

} // namespace trajectory_generator
