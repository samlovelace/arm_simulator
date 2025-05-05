from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch.substitutions import Command
import os

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Paths
    pkg_path = get_package_share_directory('arm_simulator')
    world_path = os.path.join(pkg_path, 'worlds', 'sim_world.sdf')
    urdf_path = os.path.join(pkg_path, 'urdf', 'manipulator.urdf')

    # Load robot_description from URDF
    robot_description_content = Command(['xacro ', os.path.join(pkg_path, 'models/manipulator/manipulator.urdf.xacro')])

    return LaunchDescription([

        # Start Ignition Gazebo
        ExecuteProcess(
            cmd=['ign', 'gazebo', '--verbose', world_path],
            output='screen'
        ),

        # Joint State Publisher GUI
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_content}]
        ),

        # Delay and then spawn robot into Ignition
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='ros_ign_gazebo',
                    executable='create',
                    name='spawn_entity',
                    output='screen',
                    arguments=[
                        '-name', 'manipulator',
                        '-x', '0', '-y', '0', '-z', '0.3',
                        '-topic', 'robot_description'
                    ]
                )
            ]
        )
    ])

