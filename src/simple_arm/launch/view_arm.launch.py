# In launch/view_arm.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # --- File Paths ---
    urdf_path = os.path.join(
        get_package_share_directory('simple_arm'),
        'description',
        'MarkII_urdf.urdf'
    )
    
    # --- Robot Description ---
    robot_description_config = xacro.process_file(urdf_path)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # --- Nodes ---
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        # THIS IS THE KEY CHANGE TO REDIRECT THE GUI'S OUTPUT
        remappings=[('/joint_states', '/joint_commands')]
    )

    hardware_interface_node = Node(
        package='simple_arm',
        executable='arm_hardware_interface',
        name='arm_hardware_interface',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'baudrate': 1000000,
        }]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        hardware_interface_node,
        rviz_node
    ])