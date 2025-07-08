from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    # Locate world path
    world_path = os.path.join(
        get_package_share_directory('robot_simulator'),
        'worlds',
        'UR.world'
    )

    # Prepare resource paths
    pkg_share = get_package_share_directory('robot_simulator')
    models_dir = os.path.join(pkg_share, 'models')

    ign_resource_path = os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    os.environ['IGN_GAZEBO_RESOURCE_PATH'] = models_dir + ':' + ign_resource_path

    arm_sim_path = get_package_share_directory('robot_simulator')
    os.environ['IGN_GAZEBO_RESOURCE_PATH'] = arm_sim_path + ':' + os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')

    gazebo_ign_plugin_path = os.environ.get('GAZEBO_IGN_SYSTEM_PLUGIN_PATH', '')
    os.environ['GAZEBO_IGN_SYSTEM_PLUGIN_PATH'] = (
        arm_sim_path + "/../install/robot_simulator/lib:" + gazebo_ign_plugin_path
    )

    # ROS 2 node to run ros_ign_bridge parameter_bridge
    bridge_node = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        name='ign_ros_bridge',
        output='screen',
        arguments=[
            # Left camera
            '/world/panda_world/model/stereo_camera/link/camera_link/sensor/left_camera/image'
            + '@sensor_msgs/msg/Image@ignition.msgs.Image',
            
            # Right camera
            '/world/panda_world/model/stereo_camera/link/camera_link/sensor/right_camera/image'
            + '@sensor_msgs/msg/Image@ignition.msgs.Image',
            
            # You could add more bridges here (e.g. camera_info)
        ],
        remappings=[
            # Optional: remap long Ign topics to simpler ROS names
            ('/world/panda_world/model/stereo_camera/link/camera_link/sensor/left_camera/image',
             '/stereo/left/image_raw'),
            
            ('/world/panda_world/model/stereo_camera/link/camera_link/sensor/right_camera/image',
             '/stereo/right/image_raw'),
        ]
    )

    return LaunchDescription([
        # Start Ignition Gazebo
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-v', '4', '-r', world_path],
            output='screen'
        ),

        # Start the bridge node
        bridge_node,
    ])
