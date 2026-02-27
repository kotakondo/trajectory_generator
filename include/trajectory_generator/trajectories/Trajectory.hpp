/**
 * @file Trajectory.hpp
 * @brief Abstract base class for trajectory generation (ROS2 port)
 * @author Aleix Paris (original), ported to ROS2
 * @date 2020-02-18 (original), 2026-02-27 (ROS2 port)
 */

#pragma once

#include <vector>
#include <cmath>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <Eigen/Core>

namespace trajectory_generator {

/**
 * Abstract class that represents a trajectory. All trajectories inherit from it.
 * ROS2 port: outputs PoseStamped waypoints instead of snapstack_msgs::Goal.
 */
class Trajectory
{
public:

    Trajectory(double dt) : dt_(dt) {}
    virtual ~Trajectory() {}

    /**
     * Generate the trajectory waypoints.
     * @param waypoints Output vector of PoseStamped waypoints.
     */
    virtual void generateTraj(std::vector<geometry_msgs::msg::PoseStamped>& waypoints) = 0;

    /**
     * Check if the trajectory parameters are within the given bounds.
     * For ground robot, z bounds are ignored (z=0).
     */
    virtual bool trajectoryInsideBounds(double xmin, double xmax,
                                        double ymin, double ymax) = 0;

protected:

    /**
     * Create a PoseStamped from position (x, y, 0) and yaw angle.
     * Quaternion from yaw: q.w = cos(yaw/2), q.z = sin(yaw/2), q.x = q.y = 0
     */
    static geometry_msgs::msg::PoseStamped createPoseStamped(
        double x, double y, double yaw, const std::string& frame_id = "odom")
    {
        geometry_msgs::msg::PoseStamped ps;
        ps.header.frame_id = frame_id;
        ps.pose.position.x = x;
        ps.pose.position.y = y;
        ps.pose.position.z = 0.0;  // ground robot
        ps.pose.orientation.x = 0.0;
        ps.pose.orientation.y = 0.0;
        ps.pose.orientation.z = std::sin(yaw / 2.0);
        ps.pose.orientation.w = std::cos(yaw / 2.0);
        return ps;
    }

    static bool isPointInsideBounds(double xmin, double xmax,
                                    double ymin, double ymax,
                                    double px, double py)
    {
        if (px < xmin || px > xmax) return false;
        if (py < ymin || py > ymax) return false;
        return true;
    }

    double dt_;  // goal publication period [s]
};

} // namespace trajectory_generator
