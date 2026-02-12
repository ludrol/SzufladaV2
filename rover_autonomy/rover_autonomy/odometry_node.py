"""
Odometry Node for ROS2 Jazzy.

Reads encoder data from ESP32 via MQTT and computes wheel odometry
using differential drive kinematics.

Publishes:
    - nav_msgs/Odometry on /odom
    - TF: odom -> base_link

ESP32 encoder format (every 200ms):
    "enc1_total,enc2_total,enc1_delta,enc2_delta"

Note: Encoders are direction-blind (only count up). Direction is
inferred from the last /cmd_vel command.
"""

import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

try:
    import paho.mqtt.client as mqtt
    PAHO_V2 = hasattr(mqtt, 'CallbackAPIVersion')
except ImportError:
    raise ImportError(
        "paho-mqtt is required. Install with: pip3 install paho-mqtt"
    )


def quaternion_from_yaw(yaw):
    """Create a Quaternion message from a yaw angle."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class OdometryNode(Node):
    """Computes and publishes odometry from ESP32 encoder data."""

    def __init__(self):
        super().__init__('odometry_node')

        # Declare parameters (calibrate with real measurements)
        self.declare_parameter('mqtt_broker', '192.168.1.1')
        self.declare_parameter('mqtt_port', 1883)
        self.declare_parameter('wheel_diameter', 0.065)
        self.declare_parameter('wheel_base', 0.20)
        self.declare_parameter('encoder_ticks_per_rev', 20)
        self.declare_parameter('motor_inversion', True)

        # Read parameters
        self.mqtt_broker = self.get_parameter('mqtt_broker').get_parameter_value().string_value
        self.mqtt_port = self.get_parameter('mqtt_port').get_parameter_value().integer_value
        self.wheel_diameter = self.get_parameter('wheel_diameter').get_parameter_value().double_value
        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value
        self.ticks_per_rev = self.get_parameter('encoder_ticks_per_rev').get_parameter_value().integer_value
        self.motor_inversion = self.get_parameter('motor_inversion').get_parameter_value().bool_value

        # Derived
        self.wheel_radius = self.wheel_diameter / 2.0
        self.meters_per_tick = (math.pi * self.wheel_diameter) / self.ticks_per_rev

        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        # Direction tracking (from cmd_vel)
        self.left_direction = 1.0
        self.right_direction = 1.0

        # ROS2 Publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 50)
        self.encoder_pub = self.create_publisher(String, 'encoder', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to cmd_vel to infer direction
        self.create_subscription(Twist, 'cmd_vel', self._cmd_vel_callback, 10)

        # --- MQTT Client ---
        if PAHO_V2:
            self.mqtt_client = mqtt.Client(
                callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
                client_id='ros2_odometry'
            )
        else:
            self.mqtt_client = mqtt.Client(client_id='ros2_odometry')

        self.mqtt_client.on_connect = self._on_mqtt_connect
        self.mqtt_client.on_message = self._on_mqtt_message

        try:
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, 60)
            self.mqtt_client.loop_start()
            self.get_logger().info(
                f'Odometry node connected to MQTT at {self.mqtt_broker}'
            )
        except Exception as e:
            self.get_logger().error(f'Failed to connect to MQTT: {e}')

        self.get_logger().info(
            f'Odometry params: wheel_d={self.wheel_diameter}m, '
            f'base={self.wheel_base}m, ticks/rev={self.ticks_per_rev}'
        )

    def _cmd_vel_callback(self, msg):
        """Track motor direction from cmd_vel for encoder sign inference."""
        v_left = msg.linear.x - msg.angular.z * (self.wheel_base / 2.0)
        v_right = msg.linear.x + msg.angular.z * (self.wheel_base / 2.0)

        self.left_direction = 1.0 if v_left >= 0.0 else -1.0
        self.right_direction = 1.0 if v_right >= 0.0 else -1.0

    def _on_mqtt_connect(self, client, userdata, flags, reason_code, properties=None):
        """MQTT connect callback."""
        self.get_logger().info('Odometry MQTT connected')
        client.subscribe('sensor/encoder')

    def _on_mqtt_message(self, client, userdata, msg):
        """Process encoder data from ESP32."""
        try:
            data = msg.payload.decode().strip()

            # Also republish as String for debugging
            enc_msg = String()
            enc_msg.data = data
            self.encoder_pub.publish(enc_msg)

            # Parse: "enc1_total,enc2_total,enc1_delta,enc2_delta"
            parts = data.split(',')
            if len(parts) != 4:
                return

            delta_left = abs(int(parts[2]))
            delta_right = abs(int(parts[3]))

            # Apply direction (inferred from last cmd_vel)
            if self.motor_inversion:
                # Forward = negative PWM, so invert direction inference
                d_left = delta_left * self.meters_per_tick * (-self.left_direction)
                d_right = delta_right * self.meters_per_tick * (-self.right_direction)
            else:
                d_left = delta_left * self.meters_per_tick * self.left_direction
                d_right = delta_right * self.meters_per_tick * self.right_direction

            # Differential drive odometry
            d_center = (d_left + d_right) / 2.0
            d_theta = (d_right - d_left) / self.wheel_base

            # Update pose
            current_time = self.get_clock().now()
            dt = (current_time - self.last_time).nanoseconds / 1e9
            self.last_time = current_time

            self.theta += d_theta
            self.x += d_center * math.cos(self.theta)
            self.y += d_center * math.sin(self.theta)

            # Compute velocities
            if dt > 0:
                vx = d_center / dt
                vth = d_theta / dt
            else:
                vx = 0.0
                vth = 0.0

            # Publish TF: odom -> base_link
            t = TransformStamped()
            t.header.stamp = current_time.to_msg()
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation = quaternion_from_yaw(self.theta)
            self.tf_broadcaster.sendTransform(t)

            # Publish Odometry message
            odom = Odometry()
            odom.header.stamp = current_time.to_msg()
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_link'

            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0.0
            odom.pose.pose.orientation = quaternion_from_yaw(self.theta)

            odom.twist.twist.linear.x = vx
            odom.twist.twist.angular.z = vth

            self.odom_pub.publish(odom)

        except Exception as e:
            self.get_logger().debug(f'Encoder parse error: {e}')

    def destroy_node(self):
        """Cleanup."""
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
