from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Start Gazebo Sim (Ignition)
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-v', '4', '-r', '/home/sam/dev/cpp/arm_ws/install/arm_simulator/share/arm_simulator/worlds/UR.world'],
            output='screen'
        ),
    ])
