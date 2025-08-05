# In simple_arm/launch/interactive_control.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("MarkII_urdf", package_name="MarkII_urdf_moveit_config")
        .robot_description(file_path="config/MarkII_urdf.urdf")
        .robot_description_semantic(file_path="config/MarkII_urdf.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/ros2_controllers.yaml")
        .to_moveit_configs()
    )

    # -- NEW NODE --
    # Start Robot State Publisher
    # Publishes the TF transforms for the robot's links based on joint states.
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )
    
    # Start the main MoveGroup node
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )
    
    # Start RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(get_package_share_directory("MarkII_urdf_moveit_config"), "config", "moveit.rviz")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    # Start Static TF Publisher
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    # Your custom nodes
    hardware_interface_node = Node(
        package='simple_arm',
        executable='arm_hardware_interface',
        name='arm_hardware_interface',
        output='screen'
    )

    interactive_ik_server_node = Node(
        package='simple_arm',
        executable='interactive_ik_server',
        name='interactive_ik_server',
        output='screen'
    )

    # Add the new robot_state_publisher_node to the list of nodes to launch
    return LaunchDescription([
        robot_state_publisher_node,
        run_move_group_node,
        rviz_node,
        static_tf_node,
        hardware_interface_node,
        interactive_ik_server_node,
    ])