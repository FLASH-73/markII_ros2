# In simple_arm/simple_arm/arm_hardware_interface.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from . import sts_driver
from std_srvs.srv import Trigger
import threading

class ArmHardwareInterface(Node):
    def __init__(self):
        super().__init__('arm_hardware_interface')

        self.motors_enabled = False
        self.system_primed = False
        # --- Joint Mappings ---
        self.single_servo_joints = {
            'Base_link_Revolute-1': 1, 'link3_Revolute-4': 6,
            'link4_Revolute-5': 7, 'link5_Revolute-9': 8,
            'gripper_finger_joint': 9
        }
        self.dual_servo_joints = {
            'link1_Revolute-2': [2, 3], 'link2_Revolute-3': [4, 5]
        }
        self.joint_names = list(self.single_servo_joints.keys()) + list(self.dual_servo_joints.keys())
        
        # --- NEW: Precise Calibration Data Structure ---
        # You will fill this out as you complete the calibration for each joint.
        self.calibration = {
            'Base_link_Revolute-1': {'scale': 644.9, 'offset': 2208},
            'link1_Revolute-2': {'scale': 651, 'offset': 3052},
            'link2_Revolute-3': {'scale': 647.2, 'offset': 1005},
            'link3_Revolute-4': {'scale': 628, 'offset': 2570},
            'link4_Revolute-5': {'scale': 640, 'offset': 725},
            'link5_Revolute-9': {'scale': 654, 'offset': 593},
            'gripper_finger_joint': {'scale': 29860, 'offset':3313}
        }

        self.joint_positions = [0.0] * len(self.joint_names)
        self.position_update_lock = threading.Lock()

        # --- Parameters & Driver Initialization ---
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 1000000)
        self.serial_port = self.get_parameter('serial_port').value
        self.baudrate = self.get_parameter('baudrate').value
        try:
            self.driver = sts_driver.ServoController(self.serial_port, self.baudrate)
            self.get_logger().info(f"Successfully connected on {self.serial_port}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to servos: {e}")
            rclpy.shutdown(); return

        # --- ROS Communications ---
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.joint_command_subscriber = self.create_subscription(JointState, 'joint_commands', self.command_callback, 10)
        self.enable_service = self.create_service(Trigger, 'enable_motors', self.enable_motors_callback)
        
        # --- Using two timers for clarity: one for reading, one for publishing ---
        self.hardware_read_timer = self.create_timer(0.1, self.read_hardware_state_callback) # Read hardware at 10Hz
        self.state_publish_timer = self.create_timer(0.02, self.publish_states_callback) # Publish to RViz at 50Hz

        self.get_logger().info("Arm Hardware Interface started.")
        self.get_logger().info("RViz will sync with the real robot's position.")
        self.get_logger().info("Call /enable_motors service to allow physical movement.")

    def enable_motors_callback(self, request, response):
        self.motors_enabled = True
        self.system_primed = False
        self.get_logger().info("Motors have been enabled. Physical robot will now move.")
        response.success = True
        response.message = "Motors enabled."
        return response

    # --- NEW: Precise Conversion Functions ---
    def _convert_rad_to_ticks(self, rad, joint_name):
        if joint_name in self.calibration:
            cal = self.calibration[joint_name]
            return int(cal['offset'] + rad * cal['scale'])
        else: # Fallback for uncalibrated joints
            self.get_logger().warn(f"Using fallback calibration for {joint_name}", throttle_duration_sec=5)
            return int(2048 + (rad * (2048 / 3.14159)))

    def _convert_ticks_to_rad(self, ticks, joint_name):
        if joint_name in self.calibration:
            cal = self.calibration[joint_name]
            # Avoid division by zero if scale is somehow 0
            return (ticks - cal['offset']) / cal['scale'] if cal['scale'] != 0 else 0.0
        else: # Fallback for uncalibrated joints
            return (ticks - 2048) * (3.14159 / 2048)

    # Note: Gripper conversion is linear and doesn't need the same calibration
    def _convert_gripper_dist_to_ticks(self, dist):
        min_dist, max_dist, min_ticks, max_ticks = 0.0, 0.035, 1024, 3072
        dist = max(min_dist, min(dist, max_dist))
        return int(min_ticks + ((dist - min_dist) / (max_dist - min_dist)) * (max_ticks - min_ticks))

    def _convert_ticks_to_gripper_dist(self, ticks):
        min_dist, max_dist, min_ticks, max_ticks = 0.0, 0.035, 1024, 3072
        ticks = max(min_ticks, min(ticks, max_ticks))
        return min_dist + ((ticks - min_ticks) / (max_ticks - min_ticks)) * (max_dist - min_dist)

    def read_hardware_state_callback(self):
        """Reads the current position from all servos and updates the internal state."""
        temp_positions = []
        for name in self.joint_names:
            pos_value = 0.0
            id_to_read = None
            pos_ticks = None

            if name in self.single_servo_joints:
                id_to_read = self.single_servo_joints[name]
                pos_ticks = self.driver.get_position(id_to_read)
                if pos_ticks is not None:
                    if name == 'gripper_finger_joint':
                        pos_value = self._convert_ticks_to_gripper_dist(pos_ticks)
                    else:
                        pos_value = self._convert_ticks_to_rad(pos_ticks, name)
            elif name in self.dual_servo_joints:
                id_to_read = self.dual_servo_joints[name][0]
                pos_ticks = self.driver.get_position(id_to_read)
                if pos_ticks is not None:
                    pos_value = self._convert_ticks_to_rad(pos_ticks, name)
            
            # --- IMPORTANT FOR CALIBRATION ---
            # Uncomment the block below and set the ID to watch a specific servo's raw tick value.
            #if id_to_read == 4: # Example: Watch servo ID 1 (Base)
             #    self.get_logger().info(f"ID {id_to_read} Ticks: {pos_ticks}")
            
            temp_positions.append(pos_value)
        
        with self.position_update_lock:
            self.joint_positions = temp_positions
            
    def command_callback(self, msg: JointState):
        """Receives commands and sends them to the physical hardware if enabled."""
        if not self.motors_enabled:
            self.get_logger().warn("Motors not enabled. Call /enable_motors to move the robot.", throttle_duration_sec=10)
            return

        commands_to_sync = {}
        
        # --- NEW PRIMING LOGIC ---
        if not self.system_primed:
            self.get_logger().info("First command after enable: syncing GUI to current robot state.")
            # Use the last read hardware positions as the command
            with self.position_update_lock:
                positions_to_use = self.joint_positions
            
            # Use the names from the incoming message for correct ordering
            for i, name in enumerate(self.joint_names):
                # Find the index of this joint in the positions_to_use list
                joint_index = self.joint_names.index(name)
                position_rad = positions_to_use[joint_index]

                if name == 'gripper_finger_joint':
                    position_ticks = self._convert_gripper_dist_to_ticks(position_rad)
                else:
                    position_ticks = self._convert_rad_to_ticks(position_rad, name)

                if name in self.single_servo_joints:
                    servo_id = self.single_servo_joints[name]
                    commands_to_sync[servo_id] = position_ticks
                elif name in self.dual_servo_joints:
                    leader_id, follower_id = self.dual_servo_joints[name]
                    commands_to_sync[leader_id] = position_ticks
                    commands_to_sync[follower_id] = 4095 - position_ticks
            
            self.system_primed = True # Mark as primed so we don't do this again
        else:
            # --- ORIGINAL LOGIC ---
            # This will run for all subsequent commands
            for i, name in enumerate(msg.name):
                if name == 'gripper_finger_joint':
                    position_ticks = self._convert_gripper_dist_to_ticks(msg.position[i])
                else:
                    position_ticks = self._convert_rad_to_ticks(msg.position[i], name)

                if name in self.single_servo_joints:
                    servo_id = self.single_servo_joints[name]
                    commands_to_sync[servo_id] = position_ticks
                elif name in self.dual_servo_joints:
                    leader_id, follower_id = self.dual_servo_joints[name]
                    commands_to_sync[leader_id] = position_ticks
                    commands_to_sync[follower_id] = 4095 - position_ticks
        
        if commands_to_sync:
            self.driver.sync_write_positions(commands_to_sync)

    def publish_states_callback(self):
        """Publishes the last known joint positions to /joint_states for RViz."""
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'Base_link'
        msg.name = self.joint_names
        
        with self.position_update_lock:
            msg.position = self.joint_positions

        msg.velocity = [0.0] * len(self.joint_names)
        msg.effort = [0.0] * len(self.joint_names)
        self.joint_state_publisher.publish(msg)

    def destroy_node(self):
        """Called automatically on shutdown to clean up resources."""
        self.get_logger().info("Shutdown signal received. Disabling torque on all servos.")
        
        # Gather all unique servo IDs from the joint mappings
        # The joint mappings are defined in this file's __init__ method
        all_servo_ids = set(self.single_servo_joints.values())
        for ids in self.dual_servo_joints.values():
            all_servo_ids.update(ids)
            
        for servo_id in all_servo_ids:
            try:
                # The set_torque_enable function is from the driver
                self.driver.set_torque_enable(servo_id, False)
            except Exception as e:
                self.get_logger().error(f"Failed to disable torque for servo {servo_id}: {e}")
        
        # Close the serial port connection
        self.driver.close()
        # It's important to call the parent class's method
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ArmHardwareInterface()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()