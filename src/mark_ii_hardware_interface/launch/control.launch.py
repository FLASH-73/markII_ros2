from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
# Import for IncludeLaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Declare a launch argument to control whether MoveIt is started
    # This makes your launch file more flexible
    launch_moveit_arg = DeclareLaunchArgument(
        "launch_moveit",
        default_value="true",
        description="Launch MoveIt move_group node",
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("simple_arm"), "description", "MarkII_urdf.urdf"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("mark_ii_hardware_interface"),
            "config",
            "controllers.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", PathJoinSubstitution([FindPackageShare("simple_arm"), "config", "view_arm.rviz"])],
    )

    spawn_forward_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "-c", "/controller_manager"],
        output="screen",
    )

    # The new interactive marker node that provides the draggable marker
    interactive_marker_node = Node(
        package="simple_arm",
        executable="interactive_marker_node",
        name="interactive_marker_node",
        output="screen",
    )

    # The MoveIt launch file
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('MarkII_urdf_moveit_config'),
                'launch',
                'moveit.launch.py'
            ])
        ),
        condition_if_not_equal=LaunchConfiguration('launch_moveit', default='false')
    )


    return LaunchDescription([
        launch_moveit_arg,
        control_node,
        robot_state_publisher_node,
        rviz_node,
        spawn_forward_controller,
        # Add the new nodes to the launch description
        interactive_marker_node,
        moveit_launch,
    ])
