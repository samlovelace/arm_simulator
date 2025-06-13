from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    world_path = os.path.join(
        get_package_share_directory('arm_simulator'),
        'worlds',
        'UR.world'
    )

    return LaunchDescription([
        # Start Gazebo Sim (Ignition)
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-v', '4', '-r', world_path],
            output='screen'
        ),
    ])
