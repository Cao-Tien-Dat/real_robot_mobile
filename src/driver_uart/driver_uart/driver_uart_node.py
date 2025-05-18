import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
import serial
import math
import time

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odometry_publisher')

        # Robot specs
        self.pulse_rev_left = 2970.0
        self.pulse_rev_right = 2970.0
        self.wheel_radius = 0.0275
        self.track = 0.244
        self.sprocket = 2 * math.pi * self.wheel_radius
        self.ticks_per_meter = self.pulse_rev_left / self.sprocket

        # Pose state
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        # Encoder readings
        self.encoder_left = 0
        self.encoder_right = 0
        self.prev_encoder_left = 0
        self.prev_encoder_right = 0

        # Wheel angle positions
        self.left_wheel_pos = 0.0
        self.right_wheel_pos = 0.0

        # Command state
        self.cmd_l = 0
        self.cmd_r = 0
        self.last_cmd_time = self.get_clock().now()
        self.last_time = self.get_clock().now()

        # Serial
        self.ser = serial.Serial('/dev/esp', 115200, timeout=0.002) #500hz

        # ROS2 comm
        self.odom_pub = self.create_publisher(Odometry, 'odom', 1023)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 1023)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 1023)

        # Timers
        self.create_timer(0.005, self.encoder_update)       # 100 Hz
        self.create_timer(0.005, self.publish_odometry)     # 100 Hz

    def read_encoder_values(self):
        try:
            self.ser.write(b"e\r")
            line = self.ser.readline().decode().strip()
            parts = line.split(" ")
            if len(parts) == 2:
                self.encoder_left = int(parts[0])
                self.encoder_right = int(parts[1])
        except Exception as e:
            self.get_logger().warn(f"Encoder read failed: {e}")

    def set_motor_values(self, left_val, right_val):
        left_val = max(-6, min(6, left_val))
        right_val = max(-6, min(6, right_val))
        self.cmd_l = left_val
        self.cmd_r = right_val
        command = f"m {left_val} {right_val}\r"
        self.ser.write(command.encode())

    def cmd_vel_callback(self, msg):
        v = msg.linear.x
        w = msg.angular.z
        v_l = v - w * self.track / 2.0
        v_r = v + w * self.track / 2.0
        tick_l = int(v_l * self.ticks_per_meter * 0.05)
        tick_r = int(v_r * self.ticks_per_meter * 0.05)
        self.set_motor_values(tick_l, tick_r)
        self.last_cmd_time = self.get_clock().now()

    def encoder_update(self):
        now = self.get_clock().now()
        if (now - self.last_cmd_time).nanoseconds / 1e9 > 0.5:
            if self.cmd_l != 0 or self.cmd_r != 0:
                self.set_motor_values(0, 0)
        self.read_encoder_values()

    def publish_odometry(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        delta_left = (self.encoder_left - self.prev_encoder_left) / self.ticks_per_meter
        delta_right = (self.encoder_right - self.prev_encoder_right) / self.ticks_per_meter

        self.prev_encoder_left = self.encoder_left
        self.prev_encoder_right = self.encoder_right

        delta_distance = (delta_left + delta_right) / 2.0
        delta_theta = (delta_right - delta_left) / self.track

        if abs(delta_distance) > 1e-6:
            self.x += delta_distance * math.cos(self.th + delta_theta / 2.0)
            self.y += delta_distance * math.sin(self.th + delta_theta / 2.0)
        self.th += delta_theta

        self.left_wheel_pos += delta_left / self.wheel_radius
        self.right_wheel_pos += delta_right / self.wheel_radius

        qz = math.sin(self.th / 2.0)
        qw = math.cos(self.th / 2.0)

        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = delta_distance / dt
        odom.twist.twist.angular.z = delta_theta / dt
        self.odom_pub.publish(odom)

        joint_msg = JointState()
        joint_msg.header.stamp = now.to_msg()
        joint_msg.name = ['khop_banh_trai', 'khop_banh_phai']
        joint_msg.position = [self.left_wheel_pos, self.right_wheel_pos]
        joint_msg.velocity = [
            delta_left / dt / self.wheel_radius,
            delta_right / dt / self.wheel_radius
        ]
        self.joint_pub.publish(joint_msg)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()