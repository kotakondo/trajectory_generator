"""
Launch file for the trajectory_generator package.

Launch argument:
  traj_shape (required)
    - "8":      Figure8 trajectory
    - "circle": Circle trajectory
    - "line":   Line trajectory

All shapes are handled by trajectory_generator_node.
Parameters are loaded from config/params.yaml.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the package share directory for config file path
    pkg_share = get_package_share_directory('trajectory_generator')
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')

    # Declare launch arguments
    traj_shape_arg = DeclareLaunchArgument(
        'traj_shape',
        description='traj_shape should be set to either 8, circle, or line',
        choices=['8', 'circle', 'line'],
    )
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value=EnvironmentVariable('ROVER_NAME', default_value='RR03'),
        description='Robot namespace for all topics',
    )
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='odom',
        description='Input odometry topic name (relative to namespace)',
    )

    traj_shape = LaunchConfiguration('traj_shape')

    # Map traj_shape to traj_type: "8" → "Figure8", "circle" → "Circle", "line" → "Line"
    traj_type_expr = PythonExpression([
        "'Figure8' if '", traj_shape, "' == '8' else ",
        "'Circle' if '", traj_shape, "' == 'circle' else 'Line'"
    ])

    # Trajectory generator node (handles all trajectory types)
    traj_gen_node = Node(
        package='trajectory_generator',
        executable='trajectory_generator_node',
        name='trajectory_generator_node',
        namespace=LaunchConfiguration('robot_name'),
        output='screen',
        parameters=[
            params_file,
            {'traj_type': traj_type_expr},
        ],
        remappings=[
            ('odom', LaunchConfiguration('odom_topic')),
        ],
    )

    return LaunchDescription([
        traj_shape_arg,
        robot_name_arg,
        odom_topic_arg,
        traj_gen_node,
    ])
