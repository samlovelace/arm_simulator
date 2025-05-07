import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        LogInfo(
            condition=None,
            msg="Launching Ignition Gazebo Fortress with Empty World"
        ),
        Node(
            package='ignition-gazebo',
            executable='ign gazebo',
            arguments=['-v', '4', 'empty.world'],  # Launching empty world with verbosity level 4
            output='screen'
        ),
    ])

