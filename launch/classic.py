import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('arm_simulator')
    urdf_file = PathJoinSubstitution([pkg_share, 'urdf', 'manipulator.urdf'])

    urdf_full_path = os.path.join(
        os.getenv('AMENT_PREFIX_PATH').split(':')[0],  # assumes first workspace
        'share', 'arm_simulator', 'urdf', 'manipulator.urdf'
    )

    with open(urdf_full_path, 'r') as infp:
        robot_description_content = infp.read()

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('gazebo_ros'),
                    'launch',
                    'gazebo.launch.py'
                ])
            ])
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-file', urdf_file, '-entity', 'manipulator'],
            output='screen'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description_content}]
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
        ),
    ])
