# In simple_arm/launch/interactive_control.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Get the path to the MoveIt config package
    moveit_config_pkg = get_package_share_directory('MarkII_urdf_moveit_config')

    # --- Launch MoveIt ---
    # This is the primary launch file from your MoveIt config package
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_pkg, 'launch', 'moveit_rviz.launch.py')
        )
    )

    # --- Launch Our Own Nodes ---
    # 1. Your real hardware interface
    hardware_interface_node = Node(
        package='simple_arm',
        executable='arm_hardware_interface',
        name='arm_hardware_interface',
        output='screen'
    )

    # 2. Your new interactive IK server
    interactive_ik_server_node = Node(
        package='simple_arm',
        executable='interactive_ik_server',
        name='interactive_ik_server',
        output='screen'
    )

    return LaunchDescription([
        moveit_launch,
        hardware_interface_node,
        interactive_ik_server_node
    ])