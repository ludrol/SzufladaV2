"""
cmd_vel Bridge Node for ROS2 Jazzy.

Subscribes to /cmd_vel (geometry_msgs/Twist) and converts it to
differential drive left/right PWM values, then publishes them as
ROS2 Int32 messages on /motor_left and /motor_right.

The mqtt_bridge node forwards those to the ESP32 via MQTT.
This node has ZERO MQTT dependency.

Differential drive kinematics:
    v_left  = linear.x - angular.z * (wheel_base / 2)
    v_right = linear.x + angular.z * (wheel_base / 2)
    PWM = velocity * (max_pwm / max_linear_speed)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32


class CmdVelBridgeNode(Node):
    """Converts /cmd_vel Twist messages to motor PWM commands (ROS2 only)."""

    def __init__(self):
        super().__init__('cmd_vel_bridge')

        # Declare parameters
        self.declare_parameter('max_pwm', 255)
        self.declare_parameter('max_linear_speed', 0.3)
        self.declare_parameter('wheel_base', 0.20)
        self.declare_parameter('motor_inversion', True)
        self.declare_parameter('timeout_sec', 0.5)
        self.declare_parameter('right_trim', 1.05)

        # Read parameters
        self.max_pwm = self.get_parameter(
            'max_pwm').get_parameter_value().integer_value
        self.max_linear_speed = self.get_parameter(
            'max_linear_speed').get_parameter_value().double_value
        self.wheel_base = self.get_parameter(
            'wheel_base').get_parameter_value().double_value
        self.motor_inversion = self.get_parameter(
            'motor_inversion').get_parameter_value().bool_value
        self.timeout_sec = self.get_parameter(
            'timeout_sec').get_parameter_value().double_value
        self.right_trim = self.get_parameter(
            'right_trim').get_parameter_value().double_value

        # Track last command time for safety timeout
        self.last_cmd_time = self.get_clock().now()

        # ROS2 Publishers -> mqtt_bridge will forward to ESP32
        self.motor_left_pub = self.create_publisher(Int32, 'motor_left', 10)
        self.motor_right_pub = self.create_publisher(Int32, 'motor_right', 10)

        # Subscribe to /cmd_vel
        self.create_subscription(Twist, 'cmd_vel', self._cmd_vel_callback, 10)

        # Safety timer -- stop motors if no cmd_vel received
        self.create_timer(0.1, self._safety_check)

        self.get_logger().info(
            f'cmd_vel bridge ready (no MQTT). '
            f'wheel_base={self.wheel_base}m, max_speed={self.max_linear_speed}m/s'
        )

    def _cmd_vel_callback(self, msg):
        """Convert Twist to left/right PWM and publish via ROS2."""
        self.last_cmd_time = self.get_clock().now()

        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Differential drive: compute left and right wheel velocities
        v_left = linear_x - angular_z * (self.wheel_base / 2.0)
        v_right = linear_x + angular_z * (self.wheel_base / 2.0)

        # Convert to PWM (-255 to 255)
        pwm_left = int(self._velocity_to_pwm(v_left))
        pwm_right = int(self._velocity_to_pwm(v_right) * self.right_trim)

        # Apply motor inversion if needed (forward = negative PWM on this rover)
        if self.motor_inversion:
            pwm_left = -pwm_left
            pwm_right = -pwm_right

        # Clamp
        pwm_left = max(-self.max_pwm, min(self.max_pwm, pwm_left))
        pwm_right = max(-self.max_pwm, min(self.max_pwm, pwm_right))

        # Publish via ROS2 (mqtt_bridge forwards to ESP32)
        left_msg = Int32()
        left_msg.data = pwm_left
        self.motor_left_pub.publish(left_msg)

        right_msg = Int32()
        right_msg.data = pwm_right
        self.motor_right_pub.publish(right_msg)

        self.get_logger().debug(
            f'cmd_vel: lin={linear_x:.2f} ang={angular_z:.2f} -> '
            f'PWM L={pwm_left} R={pwm_right}'
        )

    def _velocity_to_pwm(self, velocity):
        """Map a velocity (m/s) to a PWM value (-255 to 255)."""
        if self.max_linear_speed == 0.0:
            return 0.0
        return (velocity / self.max_linear_speed) * self.max_pwm

    def _safety_check(self):
        """Stop motors if no cmd_vel received within timeout."""
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > self.timeout_sec:
            left_msg = Int32()
            left_msg.data = 0
            self.motor_left_pub.publish(left_msg)

            right_msg = Int32()
            right_msg.data = 0
            self.motor_right_pub.publish(right_msg)

    def destroy_node(self):
        """Stop motors on shutdown."""
        left_msg = Int32()
        left_msg.data = 0
        self.motor_left_pub.publish(left_msg)

        right_msg = Int32()
        right_msg.data = 0
        self.motor_right_pub.publish(right_msg)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
