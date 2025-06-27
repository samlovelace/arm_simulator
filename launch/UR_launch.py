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

    pkg_share = get_package_share_directory('arm_simulator')
    models_dir = os.path.join(pkg_share, 'models')
    os.environ['IGN_GAZEBO_RESOURCE_PATH'] = models_dir + ':' + os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')

    arm_sim_path = get_package_share_directory('arm_simulator')
    os.environ['IGN_GAZEBO_RESOURCE_PATH'] = arm_sim_path + ':' + os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    os.environ['GAZEBO_IGN_SYSTEM_PLUGIN_PATH'] = arm_sim_path + "../install/arm_simulator/lib" + ':' + os.environ.get('GAZEBO_IGN_SYSTEM_PLUGIN_PATH', '') 

    return LaunchDescription([
        # Start Gazebo Sim (Ignition)
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-v', '4', '-r', world_path],
            output='screen'
        ),
    ])
