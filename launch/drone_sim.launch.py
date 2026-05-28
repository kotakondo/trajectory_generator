"""
Drone replay sim: perfect tracker + goal replay + RViz.

Starts the perfect tracker at (start_x, start_y, start_z), then the goal_replay_node
plans a min-jerk approach from there to the CSV start and streams the trajectory.
The tracker follows perfectly so you can inspect the planned path in RViz.

Example:
  ros2 launch trajectory_generator drone_sim.launch.py
  ros2 launch trajectory_generator drone_sim.launch.py csv_path:=/abs/path/to/traj.csv
  ros2 launch trajectory_generator drone_sim.launch.py start_x:=3.0 start_y:=-2.0 auto_start:=true

If auto_start:=false (default), trigger the run with:
  ros2 service call /start_replay std_srvs/srv/Trigger
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('trajectory_generator')
    default_csv = os.path.join(pkg_share, 'ref_traj',
                               'model_quadloop_to_3_2_1_back_spline10_autostretch_zplus1.csv')
    rviz_cfg = os.path.join(pkg_share, 'config', 'drone_sim.rviz')

    args = [
        DeclareLaunchArgument('csv_path', default_value=default_csv,
                              description='Absolute path to the Goal CSV to replay.'),
        DeclareLaunchArgument('start_x', default_value='3.0'),
        DeclareLaunchArgument('start_y', default_value='-2.0'),
        DeclareLaunchArgument('start_z', default_value='0.0'),
        DeclareLaunchArgument('frame_id', default_value='world'),
        DeclareLaunchArgument('auto_start', default_value='true',
                              description='Begin replay automatically after start_delay.'),
        DeclareLaunchArgument('start_delay', default_value='3.0'),
        DeclareLaunchArgument('approach_enabled', default_value='true'),
        DeclareLaunchArgument('approach_speed', default_value='1.0'),
        DeclareLaunchArgument('takeoff_z', default_value='2.0'),
        DeclareLaunchArgument('rate_scale', default_value='1.0'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
    ]

    start_pos = PythonExpression([
        "[", LaunchConfiguration('start_x'), ", ",
        LaunchConfiguration('start_y'), ", ",
        LaunchConfiguration('start_z'), "]"
    ])

    tracker = Node(
        package='trajectory_generator',
        executable='perfect_tracker_node',
        name='perfect_tracker_node',
        output='screen',
        parameters=[{
            'start_pos': start_pos,
            'frame_id': LaunchConfiguration('frame_id'),
            'child_frame': 'base_link',
            'pose_topic': 'world',
            'pub_rate': 100.0,
        }],
    )

    replay = Node(
        package='trajectory_generator',
        executable='goal_replay_node',
        name='goal_replay_node',
        output='screen',
        parameters=[{
            'csv_path': LaunchConfiguration('csv_path'),
            'topic': 'goal',
            'pose_topic': 'world',
            'frame_id': LaunchConfiguration('frame_id'),
            'rate_scale': LaunchConfiguration('rate_scale'),
            'auto_start': LaunchConfiguration('auto_start'),
            'start_delay': LaunchConfiguration('start_delay'),
            'approach_enabled': LaunchConfiguration('approach_enabled'),
            'approach_speed': LaunchConfiguration('approach_speed'),
            'takeoff_z': LaunchConfiguration('takeoff_z'),
        }],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        output='screen',
    )

    return LaunchDescription(args + [tracker, replay, rviz])
