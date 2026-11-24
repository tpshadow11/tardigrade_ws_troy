"""Run a behavior tree mission.

    ros2 launch tardigrade_bt bt_mission.launch.py tree:=playground
    ros2 launch tardigrade_bt bt_mission.launch.py tree:=playground dry_run:=false
    ros2 launch tardigrade_bt bt_mission.launch.py tree:=dive_test main_tree:=MyFirst

dry_run defaults to true: the tree is loaded and printed but nothing moves.
Arm the robot yourself before running with dry_run:=false.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'tree', default_value='playground',
            description='Tree name in tardigrade_bt/trees, or a path to an XML file'),
        DeclareLaunchArgument(
            'main_tree', default_value='',
            description='Tree ID to run when the file holds several trees (empty: the main one)'),
        DeclareLaunchArgument(
            'dry_run', default_value='true',
            description='true: load and print the tree only'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='true for Unity, false on the real robot'),
        Node(
            package='tardigrade_bt',
            executable='mission_runner',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'tree': LaunchConfiguration('tree'),
                'main_tree': LaunchConfiguration('main_tree'),
                'dry_run': LaunchConfiguration('dry_run'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),
    ])
