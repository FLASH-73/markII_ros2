import rclpy
from rclpy.node import Node
from interactive_markers import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Float64MultiArray
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import time

class InteractiveMarkerNode(Node):
    def __init__(self):
        super().__init__('interactive_marker_node')

        # The publisher now sends to the forward_position_controller's command topic
        self.command_publisher = self.create_publisher(
            Float64MultiArray, 
            "/forward_position_controller/commands", 
            10
        )
        
        # Client for MoveIt's IK service
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")
        while not self.ik_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('IK service not available, waiting again...')

        # Standard TF setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # The interactive marker server
        self.marker_server = InteractiveMarkerServer(self, "interactive_marker_server")
        
        # Throttling for performance
        self.last_request_time = self.get_clock().now()
        self.throttle_period = rclpy.duration.Duration(seconds=0.05) # 20 Hz

        # Create the marker once the node is ready
        self.create_interactive_marker()
        self.get_logger().info("Interactive Marker Node started. Drag the marker in RViz to control the arm.")

    def create_interactive_marker(self):
        """Creates and initializes the interactive marker in RViz."""
        initial_pose = self.get_current_gripper_pose()

        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "base_link"
        int_marker.name = "ik_handle"
        int_marker.description = "IK Control Handle"
        int_marker.pose = initial_pose
        int_marker.scale = 0.2

        # Create a 6-DOF control handle
        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 1.0
        control.orientation.y = 0.0
        control.orientation.z = 0.0
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)
        control = control.copy()
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 1.0
        control.orientation.z = 0.0
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)
        control = control.copy()
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 0.0
        control.orientation.z = 1.0
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)
        control = control.copy()
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        self.marker_server.insert(int_marker, feedback_callback=self.process_feedback)
        self.marker_server.applyChanges()

    def get_current_gripper_pose(self):
        """Looks up the current transform of the gripper to initialize the marker."""
        initial_pose = Pose()
        initial_pose.orientation.w = 1.0
        
        self.get_logger().info("Waiting for transform from 'base_link' to 'link6'...")
        try:
            # Wait for the transform to be available
            transform = self.tf_buffer.lookup_transform('base_link', 'link6', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=5.0))
            initial_pose.position.x = transform.transform.translation.x
            initial_pose.position.y = transform.transform.translation.y
            initial_pose.position.z = transform.transform.translation.z
            initial_pose.orientation = transform.transform.rotation
            self.get_logger().info("Success! Marker initialized to current gripper pose.")
        except tf2_ros.TransformException as ex:
            self.get_logger().error(f'Could not get transform: {ex}')
            self.get_logger().info("Using fallback pose for marker.")
            initial_pose.position.x = 0.3
            initial_pose.position.z = 0.3
        
        return initial_pose

    def process_feedback(self, feedback: InteractiveMarkerFeedback):
        """Callback for marker interaction. Throttles requests to the IK solver."""
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            now = self.get_clock().now()
            if now - self.last_request_time > self.throttle_period:
                self.last_request_time = now
                self.call_ik_solver(feedback.pose)

    def call_ik_solver(self, target_pose):
        """Sends an asynchronous request to the IK solver."""
        req = GetPositionIK.Request()
        req.ik_request.group_name = "simple_arm"
        req.ik_request.avoid_collisions = True
        
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "base_link"
        pose_stamped.pose = target_pose
        req.ik_request.pose_stamped = pose_stamped

        future = self.ik_client.call_async(req)
        future.add_done_callback(self.ik_response_callback)

    def ik_response_callback(self, future):
        """Processes the IK solution and publishes commands."""
        response = future.result()
        if response.error_code.val == response.error_code.SUCCESS:
            # The forward_position_controller expects a Float64MultiArray
            command_msg = Float64MultiArray()
            command_msg.data = response.solution.joint_state.position
            self.command_publisher.publish(command_msg)
        else:
            # Log the error but don't spam the console
            self.get_logger().warn(
                f"IK solution failed with error code: {response.error_code.val}",
                throttle_duration_sec=1.0
            )

def main(args=None):
    rclpy.init(args=args)
    node = InteractiveMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()