from launch import LaunchDescription
from launch.actions import RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Make libpython symbols available to C‐extensions (termios, etc)
    preload_py = SetEnvironmentVariable(
        name='LD_PRELOAD',
        value='/usr/lib/x86_64-linux-gnu/libpython3.12.so'
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

    # Delay RViz start until the controller_manager is ready
    delay_rviz_after_control_node = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=control_node,
            on_exit=[rviz_node],
        )
    )

    return LaunchDescription([
        preload_py,
        control_node,
        robot_state_publisher_node,
        rviz_node,
    ])
