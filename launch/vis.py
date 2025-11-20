from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('robot_simulator')
    rviz_config_file = os.path.join(pkg_share, 'models/ur5_rg2/', 'ur5.rviz')

    return LaunchDescription([
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': open(
                os.path.join(pkg_share, 'models/ur5_rg2/', 'manipulator.urdf')).read()}],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        ),
    ])
