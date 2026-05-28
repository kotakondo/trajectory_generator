"""
Hardware launch for the goal_replay_node.

Streams a pre-generated dynus_interfaces/Goal trajectory (CSV) to the "goal"
topic for an outer-loop / dynus-mighty consumer to track on the real drone.
Everything is namespaced under robot_name, e.g. robot_name:=PX01 publishes on
/PX01/goal and subscribes to the drone pose (geometry_msgs/PoseStamped) on /PX01/world.

Before flying the CSV, the node plans a two-phase min-jerk approach from the
drone's current measured pose: (1) vertical takeoff in place to takeoff_z, then
(2) horizontal transit to the CSV start point.

Typical use (trigger manually once the drone is armed and hovering/on the ground):
  ros2 launch trajectory_generator goal_replay.launch.py robot_name:=PX01 csv_path:=/abs/path/to/traj.csv
  ros2 service call /PX01/start_replay std_srvs/srv/Trigger

Launch arguments:
  csv_path          Absolute path to the Goal CSV to replay.
  robot_name        Namespace for the node, the goal topic, and the pose topic.
  frame_id          Header frame for the published Goals (match the outer loop's world frame).
  pose_topic        Topic (relative to robot_name) carrying geometry_msgs/PoseStamped for approach planning.
  rate_scale        Playback speed multiplier (1.0 = real time).
  auto_start        If true, start automatically after start_delay; else wait for the trigger service.
  start_delay       Seconds to wait before auto-starting (lets a state message arrive first).
  approach_enabled  Plan the takeoff+transit approach (true) or jump straight into the CSV (false).
  approach_speed    Cruise speed used to size the approach duration (m/s).
  approach_min_time Minimum duration of each approach phase (s).
  takeoff_z         Altitude to climb to in place before transiting (m).
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('trajectory_generator')
    default_csv = os.path.join(pkg_share, 'trajectories',
                               'model_quadloop_to_3_2_1_back_spline10_autostretch.csv')

    args = [
        DeclareLaunchArgument('csv_path', default_value=default_csv,
                              description='Absolute path to the Goal CSV to replay.'),
        DeclareLaunchArgument('robot_name', default_value='PX01',
                              description='Namespace for the node, goal topic, and pose topic.'),
        DeclareLaunchArgument('frame_id', default_value='world',
                              description='Header frame_id for published Goal messages.'),
        DeclareLaunchArgument('pose_topic', default_value='world',
                              description='Pose topic (relative to robot_name) for approach planning, e.g. /PX01/world.'),
        DeclareLaunchArgument('rate_scale', default_value='1.0'),
        DeclareLaunchArgument('auto_start', default_value='false',
                              description='Start automatically after start_delay instead of waiting for the trigger.'),
        DeclareLaunchArgument('start_delay', default_value='3.0'),
        DeclareLaunchArgument('approach_enabled', default_value='true'),
        DeclareLaunchArgument('approach_speed', default_value='1.0'),
        DeclareLaunchArgument('approach_min_time', default_value='2.0'),
        DeclareLaunchArgument('takeoff_z', default_value='2.0'),
    ]

    replay_node = Node(
        package='trajectory_generator',
        executable='goal_replay_node',
        name='goal_replay_node',
        namespace=LaunchConfiguration('robot_name'),
        output='screen',
        parameters=[{
            'csv_path': LaunchConfiguration('csv_path'),
            'topic': 'goal',
            'pose_topic': LaunchConfiguration('pose_topic'),
            'frame_id': LaunchConfiguration('frame_id'),
            'rate_scale': LaunchConfiguration('rate_scale'),
            'auto_start': LaunchConfiguration('auto_start'),
            'start_delay': LaunchConfiguration('start_delay'),
            'approach_enabled': LaunchConfiguration('approach_enabled'),
            'approach_speed': LaunchConfiguration('approach_speed'),
            'approach_min_time': LaunchConfiguration('approach_min_time'),
            'takeoff_z': LaunchConfiguration('takeoff_z'),
        }],
    )

    return LaunchDescription(args + [replay_node])
