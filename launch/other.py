from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Start Gazebo Sim (Ignition)
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-v', '4', '-r', '/home/sam/dev/cpp/arm_ws/install/arm_simulator/share/arm_simulator/worlds/empty.world'],
            output='screen'
        ),

        # Spawn your robot model in Ignition Gazebo
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'my_robot',
                '-file', '/home/sam/dev/cpp/arm_ws/install/arm_simulator/share/arm_simulator/urdf/manipulator.urdf',
                '-x', '0.0', '-y', '0.0', '-z', '1.015'
            ],
            output='screen'
        )
    ])
