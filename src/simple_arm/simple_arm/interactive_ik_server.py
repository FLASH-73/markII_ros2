# In simple_arm/simple_arm/interactive_ik_server.py
import rclpy
from rclpy.node import Node
from interactive_markers import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import time

class InteractiveIKServer(Node):
    def __init__(self):
        super().__init__('interactive_ik_server')

        self.command_publisher = self.create_publisher(JointState, "/joint_commands", 10)
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")
        while not self.ik_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('IK service not available, waiting again...')

        self.marker_server = InteractiveMarkerServer(self, "ik_server")
        self.get_logger().info("Interactive IK Server started.")
        self.create_interactive_marker()

    def create_interactive_marker(self):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "Base_link" # Your robot's base frame
        int_marker.name = "ik_handle"
        int_marker.description = "IK Control"
        int_marker.pose.position.x = 0.3
        int_marker.pose.position.y = 0.0
        int_marker.pose.position.z = 0.3
        int_marker.pose.orientation.w = 1.0
        int_marker.scale = 0.2

        # 6-DOF controls
        control = InteractiveMarkerControl()
        control.orientation.w = 1.0; control.orientation.x = 1.0; control.orientation.y = 0.0; control.orientation.z = 0.0
        control.name = "move_x"; control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)
        control = InteractiveMarkerControl()
        control.orientation.w = 1.0; control.orientation.x = 0.0; control.orientation.y = 1.0; control.orientation.z = 0.0
        control.name = "move_z"; control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)
        control = InteractiveMarkerControl()
        control.orientation.w = 1.0; control.orientation.x = 0.0; control.orientation.y = 0.0; control.orientation.z = 1.0
        control.name = "move_y"; control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)
        control = InteractiveMarkerControl()
        control.orientation.w = 1.0; control.orientation.x = 1.0; control.orientation.y = 0.0; control.orientation.z = 0.0
        control.name = "rotate_x"; control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)
        control = InteractiveMarkerControl()
        control.orientation.w = 1.0; control.orientation.x = 0.0; control.orientation.y = 1.0; control.orientation.z = 0.0
        control.name = "rotate_z"; control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)
        control = InteractiveMarkerControl()
        control.orientation.w = 1.0; control.orientation.x = 0.0; control.orientation.y = 0.0; control.orientation.z = 1.0
        control.name = "rotate_y"; control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)

        self.marker_server.insert(int_marker, self.process_feedback)
        self.marker_server.apply_changes()

    def process_feedback(self, feedback: InteractiveMarkerFeedback):
        if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            self.get_logger().info(f"Marker moved to pose: {feedback.pose}")
            self.call_ik_solver(feedback.pose)

    def call_ik_solver(self, target_pose):
        req = GetPositionIK.Request()
        req.ik_request.group_name = "simple_arm"
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "Base_link"
        pose_stamped.pose = target_pose
        req.ik_request.pose_stamped = pose_stamped

        future = self.ik_client.call_async(req)
        future.add_done_callback(self.ik_response_callback)

    def ik_response_callback(self, future):
        response = future.result()
        if response.error_code.val == response.error_code.SUCCESS:
            self.get_logger().info("IK solution found, publishing to /joint_commands.")
            self.command_publisher.publish(response.solution.joint_state)
        else:
            self.get_logger().error(f"IK solution failed with error code: {response.error_code.val}")

def main(args=None):
    rclpy.init(args=args)
    node = InteractiveIKServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()