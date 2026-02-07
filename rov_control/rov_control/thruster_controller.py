import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16MultiArray, Bool
import time

class ROVThrusterController(Node):
    def __init__(self):
        super().__init__("rov_thruster_controller")

        # --- CONFIGURATION (BIDIRECTIONAL) ---
        self.PWM_NEUTRAL = 1500  # Stopped
        self.PWM_MIN     = 1100  # Full Reverse
        self.PWM_MAX     = 1900  # Full Forward
        
        # --- STATE ---
        self.surge = 0.0
        self.yaw = 0.0
        self.heave = 0.0
        
        # Safety & Features
        self.is_armed = False         
        self.emergency_stop = False   
        self.depth_lock_active = False
        self.locked_heave_val = 0.0

        # --- SUBSCRIBERS ---
        self.cmd_sub = self.create_subscription(Twist, "/rov/cmd_vel", self.cmd_callback, 10)
        self.arm_sub = self.create_subscription(Bool, "/rov/arm_status", self.arm_callback, 10)
        self.lock_sub = self.create_subscription(Bool, "/rov/depth_lock", self.lock_callback, 10)
        self.stop_sub = self.create_subscription(Bool, "/rov/emergency_stop", self.stop_callback, 10)
        
        self.pwm_pub = self.create_publisher(Int16MultiArray, "/rov/thruster_pwm", 10)
        self.timer = self.create_timer(0.05, self.update_mixer)
        self.get_logger().info("ROV Controller: BIDIRECTIONAL MODE (1500 Center)")

    def cmd_callback(self, msg):
        self.surge = msg.linear.x
        self.yaw = msg.angular.z
        if not self.depth_lock_active:
            self.heave = msg.linear.z

    def arm_callback(self, msg):
        self.is_armed = msg.data
        if not self.is_armed: self.get_logger().info("Status: DISARMED")

    def lock_callback(self, msg):
        if self.emergency_stop or not self.is_armed: return 
        if msg.data and not self.depth_lock_active:
            self.depth_lock_active = True
            self.locked_heave_val = self.heave
            self.get_logger().info(f"🔒 DEPTH LOCK: Holding Thrust {self.locked_heave_val:.2f}")
        elif not msg.data:
            self.depth_lock_active = False

    def stop_callback(self, msg):
        self.emergency_stop = msg.data
        if self.emergency_stop:
            self.get_logger().fatal("🚨 EMERGENCY STOP ACTIVATED 🚨")
            self.is_armed = False
            self.depth_lock_active = False

    def map_pwm(self, val):
        """ 
        Maps -1.0 to 1.0 (Joystick) -> PWM_MIN to PWM_MAX (Motors)
        0.0 becomes 1500 (Neutral)
        """
        # Clamp value between -1.0 and 1.0 just in case
        val = max(-1.0, min(1.0, val))
        
        if val > 0:
            return int(self.PWM_NEUTRAL + (val * (self.PWM_MAX - self.PWM_NEUTRAL)))
        else:
            # For negative values, map 0 to -1 -> Neutral to Min
            return int(self.PWM_NEUTRAL + (val * (self.PWM_NEUTRAL - self.PWM_MIN)))

    def update_mixer(self):
        # Safety Check
        if self.emergency_stop or not self.is_armed:
            self.publish_stop()
            return

        # Mixing
        curr_heave = self.locked_heave_val if self.depth_lock_active else self.heave
        
        # Horizontal Mixing (Differential)
        h_left  = self.surge + (self.yaw * 0.5)
        h_right = self.surge - (self.yaw * 0.5)
        
        # Vertical Mixing (Now supports Up AND Down)
        # We NO LONGER check "if curr_heave < 0". Both directions are allowed.
        v_thrust = curr_heave 

        # Normalize (Keep everything within -1.0 to 1.0)
        raw = [h_left, h_right, v_thrust, v_thrust]
        max_abs = max([abs(x) for x in raw]) if raw else 0.0
        
        if max_abs > 1.0:
            raw = [x / max_abs for x in raw]

        # Convert to PWM
        final_pwm = [self.map_pwm(x) for x in raw]
        
        # Publish
        msg = Int16MultiArray()
        msg.data = final_pwm
        self.pwm_pub.publish(msg)
        
        # Logging
        lock_str = "🔒" if self.depth_lock_active else " "
        self.get_logger().info(
            f"{lock_str} S:{self.surge:.1f} Y:{self.yaw:.1f} H:{curr_heave:.1f} >>> L:{final_pwm[0]} R:{final_pwm[1]} V:{final_pwm[2]}"
        )

    def publish_stop(self):
        # BIDIRECTIONAL STOP IS 1500, NOT 1000!
        msg = Int16MultiArray()
        msg.data = [1500, 1500, 1500, 1500] 
        self.pwm_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ROVThrusterController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()