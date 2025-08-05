# In simple_arm/simple_arm/interactive_ik_server.py

import rclpy
from rclpy.node import Node
from interactive_markers import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback , Marker
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped
import time
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from std_srvs.srv import Trigger

class InteractiveIKServer(Node):
    HOME_POSITION = {
        'base_link_Revolute-1': 0.0,
        'link1_Revolute-3': 0.0,
        'link2_Revolute-4': 0.0,
        'link3_Revolute-5': 0.0,
        'link4_Revolute-6': 0.0,
        'link5_Revolute-7': 0.0,
        'link6_Slider-8': 0.0
    }
    
    def __init__(self):
        super().__init__('interactive_ik_server')

        self.command_publisher = self.create_publisher(JointState, "/joint_commands", 10)
        self.home_service = self.create_service(
            Trigger, 
            'return_to_home', 
            self.return_to_home_callback
        )
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")
        while not self.ik_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('IK service not available, waiting again...')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.marker_server = InteractiveMarkerServer(self, "ik_server")
        self.get_logger().info("Interactive IK Server started.")
        self.create_interactive_marker()

        self.last_request_time = self.get_clock().now()
        self.throttle_period = rclpy.duration.Duration(seconds=0.02)

        # FIX: Initialize an instance variable to hold the timer object.
        self._marker_update_timer = None

    def return_to_home_callback(self, request, response):
        """Service callback to send the robot to its predefined home position."""
        self.get_logger().info("Return to home service called.")
        
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = list(self.HOME_POSITION.keys())
        joint_state_msg.position = list(self.HOME_POSITION.values())
        self.command_publisher.publish(joint_state_msg)
        
        # FIX: Properly manage the one-shot timer.
        # If a timer is already active, cancel it before creating a new one.
        if self._marker_update_timer is not None:
            self._marker_update_timer.cancel()
        
        # Create the timer and store its handle in our instance variable.
        self._marker_update_timer = self.create_timer(3.0, self.timer_update_marker_callback)
        
        response.success = True
        response.message = "Command to return to home position has been sent."
        
        return response

    # ... (create_interactive_marker, process_feedback, call_ik_solver are unchanged) ...
    def create_interactive_marker(self):
        initial_pose = Pose()
        initial_pose.orientation.w = 1.0 # Set a default valid orientation

        self.get_logger().info("Waiting for transform from 'base_link' to 'link6'...")
        try:
            # Wait up to 5 seconds for the transform to become available
            for _ in range(50): # 50 tries * 0.1s sleep = 5 seconds max wait
                if self.tf_buffer.can_transform('base_link', 'link6', rclpy.time.Time()):
                    break
                time.sleep(0.1)
            
            # Once available, look up the transform
            t = self.tf_buffer.lookup_transform('base_link', 'link6', rclpy.time.Time())
            
            # Apply the transform to the initial pose
            initial_pose.position.x = t.transform.translation.x
            initial_pose.position.y = t.transform.translation.y
            initial_pose.position.z = t.transform.translation.z
            initial_pose.orientation = t.transform.rotation
            self.get_logger().info("Success! Marker initialized to current gripper pose.")

        except tf2_ros.TransformException as ex:
            self.get_logger().error(f'Could not get transform after waiting: {ex}')
            self.get_logger().info("Using fallback pose for marker.")
            # Fallback to a default pose if the transform fails
            initial_pose.position.x = 0.3
            initial_pose.position.y = 0.0
            initial_pose.position.z = 0.3

        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "base_link"
        int_marker.name = "ik_handle"
        int_marker.description = "IK Control"
        int_marker.pose = initial_pose
        int_marker.scale = 0.15 # Adjusted for a reasonable size

        # --- This is where all your "control = InteractiveMarkerControl()" blocks go ---
        # --- (No changes needed to the controls themselves) ---

        # 6-DOF controls for axes
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
        
        # Control for the central sphere
        control = InteractiveMarkerControl()
        control.interaction_mode = InteractiveMarkerControl.MOVE_3D
        control.always_visible = True
        control.name = 'move_sphere'
        sphere_marker = Marker()
        sphere_marker.type = Marker.SPHERE
        sphere_marker.scale.x = 0.025
        sphere_marker.scale.y = 0.025
        sphere_marker.scale.z = 0.025
        sphere_marker.color.r = 0.8
        sphere_marker.color.g = 0.1
        sphere_marker.color.b = 0.1
        sphere_marker.color.a = 0.7
        control.markers.append(sphere_marker)
        int_marker.controls.append(control)

        self.marker_server.insert(int_marker, feedback_callback=self.process_feedback)
        self.marker_server.applyChanges()

    def process_feedback(self, feedback: InteractiveMarkerFeedback):
    # We only process updates while the mouse button is down
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            now = self.get_clock().now()
            # Check if enough time has passed since the last request
            if now - self.last_request_time > self.throttle_period:
                self.last_request_time = now
                self.call_ik_solver(feedback.pose)

    def call_ik_solver(self, target_pose):
        req = GetPositionIK.Request()
        req.ik_request.group_name = "simple_arm"
        req.ik_request.avoid_collisions = True
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "base_link"
        pose_stamped.pose = target_pose
        req.ik_request.pose_stamped = pose_stamped

        future = self.ik_client.call_async(req)
        future.add_done_callback(self.ik_response_callback)

    def update_marker_to_home_pose(self):
        """Waits for the robot to move home, then updates the marker's pose."""
        self.get_logger().info("Updating marker pose to match robot's home position...")
        try:
            t = self.tf_buffer.lookup_transform('base_link', 'link6', rclpy.time.Time())
            
            new_pose = Pose()
            new_pose.position.x = t.transform.translation.x
            new_pose.position.y = t.transform.translation.y
            new_pose.position.z = t.transform.translation.z
            new_pose.orientation = t.transform.rotation
            
            self.marker_server.setPose("ik_handle", new_pose)
            self.marker_server.applyChanges()
            self.get_logger().info("Successfully updated marker to home position.")

        except tf2_ros.TransformException as ex:
            self.get_logger().error(f"Failed to look up home pose after homing command: {ex}")
    
    def timer_update_marker_callback(self):
        """
        FIX: This callback is for a one-shot timer. It performs its action
        and then ensures the timer is destroyed.
        """
        # First, perform the main logic.
        self.update_marker_to_home_pose()

        # Now, clean up the timer that called this callback.
        if self._marker_update_timer is not None:
            self.destroy_timer(self._marker_update_timer)
            self._marker_update_timer = None

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