from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['ign', 'gazebo', '--verbose', 'worlds/sim_world.sdf'],
            output='screen'
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            parameters=[{'use_gui': True}],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(
                '/home/sam/dev/cpp/arm_ws/install/arm_simulator/share/arm_simulator/urdf/manipulator.urdf').read()}],
        ),
        Node(
            package='ros_gz_sim',  # or 'ros_ign_gazebo' depending on your version
            executable='create',
            arguments=[
                '-name', 'three_link_manipulator',
                '-topic', 'robot_description',
                '-x', '0', '-y', '0', '-z', '0.1'
            ],
            output='screen'
        )
    ])

