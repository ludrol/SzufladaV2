"""
cmd_vel Bridge Node for ROS2 Jazzy.

Subscribes to /cmd_vel (geometry_msgs/Twist) and converts it to
differential drive left/right PWM values, then publishes via MQTT
to the ESP32.

This node is the standard interface between any ROS2 autonomy stack
(Nav2, obstacle avoidance, visual tracking, teleop) and the ESP32 motors.

Differential drive kinematics:
    v_left  = linear.x - angular.z * (wheel_base / 2)
    v_right = linear.x + angular.z * (wheel_base / 2)
    PWM = velocity * (max_pwm / max_linear_speed)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

try:
    import paho.mqtt.client as mqtt
    PAHO_V2 = hasattr(mqtt, 'CallbackAPIVersion')
except ImportError:
    raise ImportError(
        "paho-mqtt is required. Install with: pip3 install paho-mqtt"
    )


class CmdVelBridgeNode(Node):
    """Converts /cmd_vel Twist messages to MQTT motor commands."""

    def __init__(self):
        super().__init__('cmd_vel_bridge')

        # Declare parameters
        self.declare_parameter('mqtt_broker', '192.168.1.1')
        self.declare_parameter('mqtt_port', 1883)
        self.declare_parameter('max_pwm', 255)
        self.declare_parameter('max_linear_speed', 0.3)
        self.declare_parameter('wheel_base', 0.20)
        self.declare_parameter('motor_inversion', True)
        self.declare_parameter('timeout_sec', 0.5)

        # Read parameters
        self.mqtt_broker = self.get_parameter('mqtt_broker').get_parameter_value().string_value
        self.mqtt_port = self.get_parameter('mqtt_port').get_parameter_value().integer_value
        self.max_pwm = self.get_parameter('max_pwm').get_parameter_value().integer_value
        self.max_linear_speed = self.get_parameter('max_linear_speed').get_parameter_value().double_value
        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value
        self.motor_inversion = self.get_parameter('motor_inversion').get_parameter_value().bool_value
        self.timeout_sec = self.get_parameter('timeout_sec').get_parameter_value().double_value

        # MQTT Topics
        self.topic_motor_left = 'motor/left'
        self.topic_motor_right = 'motor/right'

        # Track last command time for safety timeout
        self.last_cmd_time = self.get_clock().now()

        # --- MQTT Client ---
        if PAHO_V2:
            self.mqtt_client = mqtt.Client(
                callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
                client_id='ros2_cmd_vel_bridge'
            )
        else:
            self.mqtt_client = mqtt.Client(client_id='ros2_cmd_vel_bridge')

        try:
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, 60)
            self.mqtt_client.loop_start()
            self.get_logger().info(
                f'cmd_vel bridge connected to MQTT at {self.mqtt_broker}:{self.mqtt_port}'
            )
        except Exception as e:
            self.get_logger().error(f'Failed to connect to MQTT: {e}')
            return

        # Subscribe to /cmd_vel
        self.create_subscription(Twist, 'cmd_vel', self._cmd_vel_callback, 10)

        # Safety timer -- stop motors if no cmd_vel received
        self.create_timer(0.1, self._safety_check)

        self.get_logger().info('cmd_vel bridge ready.')

    def _cmd_vel_callback(self, msg):
        """Convert Twist to left/right PWM and send via MQTT."""
        self.last_cmd_time = self.get_clock().now()

        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Differential drive: compute left and right wheel velocities
        v_left = linear_x - angular_z * (self.wheel_base / 2.0)
        v_right = linear_x + angular_z * (self.wheel_base / 2.0)

        # Convert to PWM (-255 to 255)
        pwm_left = int(self._velocity_to_pwm(v_left))
        pwm_right = int(self._velocity_to_pwm(v_right))

        # Apply motor inversion if needed (forward = negative PWM on this rover)
        if self.motor_inversion:
            pwm_left = -pwm_left
            pwm_right = -pwm_right

        # Clamp
        pwm_left = max(-self.max_pwm, min(self.max_pwm, pwm_left))
        pwm_right = max(-self.max_pwm, min(self.max_pwm, pwm_right))

        # Send via MQTT
        self.mqtt_client.publish(self.topic_motor_left, str(pwm_left))
        self.mqtt_client.publish(self.topic_motor_right, str(pwm_right))

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
            self.mqtt_client.publish(self.topic_motor_left, '0')
            self.mqtt_client.publish(self.topic_motor_right, '0')

    def destroy_node(self):
        """Stop motors and disconnect MQTT on shutdown."""
        self.mqtt_client.publish(self.topic_motor_left, '0')
        self.mqtt_client.publish(self.topic_motor_right, '0')
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()
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
