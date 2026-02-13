"""
MQTT Bridge Node for ROS2 Jazzy -- Szuflada V2.

THE SINGLE MQTT CONNECTION for the entire ROS2 stack.
All other nodes communicate through ROS2 topics only.

MQTT -> ROS2:
    sensor/distance  ->  /distance  (std_msgs/Float32)
                     ->  /range     (sensor_msgs/Range)
    sensor/encoder   ->  /encoder_raw (std_msgs/String)

ROS2 -> MQTT:
    /motor_left  (std_msgs/Int32)   ->  motor/left
    /motor_right (std_msgs/Int32)   ->  motor/right
    /led_cmd     (std_msgs/String)  ->  sensor/led
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32, String
from sensor_msgs.msg import Range

try:
    import paho.mqtt.client as mqtt
    PAHO_V2 = hasattr(mqtt, 'CallbackAPIVersion')
except ImportError:
    raise ImportError(
        "paho-mqtt is required. Install with: pip3 install paho-mqtt"
    )


class MqttBridgeNode(Node):
    """Single MQTT connection bridging ESP32 <-> ROS2."""

    def __init__(self):
        super().__init__('mqtt_bridge')

        # Parameters
        self.declare_parameter('mqtt_broker', '192.168.1.1')
        self.declare_parameter('mqtt_port', 1883)

        self.mqtt_broker = self.get_parameter(
            'mqtt_broker').get_parameter_value().string_value
        self.mqtt_port = self.get_parameter(
            'mqtt_port').get_parameter_value().integer_value

        # ── ROS2 Publishers (MQTT -> ROS2) ──────────────────────────
        self.distance_pub = self.create_publisher(Float32, 'distance', 10)
        self.range_pub = self.create_publisher(Range, 'range', 10)
        self.encoder_pub = self.create_publisher(String, 'encoder_raw', 10)

        # ── ROS2 Subscribers (ROS2 -> MQTT) ─────────────────────────
        self.create_subscription(
            Int32, 'motor_left', self._motor_left_cb, 10)
        self.create_subscription(
            Int32, 'motor_right', self._motor_right_cb, 10)
        self.create_subscription(
            String, 'led_cmd', self._led_cmd_cb, 10)

        # ── Single MQTT Client ──────────────────────────────────────
        if PAHO_V2:
            self.mqtt_client = mqtt.Client(
                callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
                client_id='ros2_mqtt_bridge'
            )
        else:
            self.mqtt_client = mqtt.Client(client_id='ros2_mqtt_bridge')

        self.mqtt_client.on_connect = self._on_mqtt_connect
        self.mqtt_client.on_message = self._on_mqtt_message
        self.mqtt_client.on_disconnect = self._on_mqtt_disconnect

        try:
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, 60)
            self.mqtt_client.loop_start()
            self.get_logger().info(
                f'MQTT bridge connected to {self.mqtt_broker}:{self.mqtt_port}'
            )
        except Exception as e:
            self.get_logger().error(f'MQTT connection failed: {e}')
            return

        self.get_logger().info('MQTT bridge ready (single connection mode).')

    # ── MQTT Callbacks ──────────────────────────────────────────────

    def _on_mqtt_connect(self, client, userdata, flags,
                         reason_code, properties=None):
        """Subscribe to all ESP32 sensor topics."""
        self.get_logger().info(f'MQTT connected (reason: {reason_code})')
        client.subscribe('sensor/distance')
        client.subscribe('sensor/encoder')
        self.get_logger().info(
            'Subscribed to: sensor/distance, sensor/encoder')

    def _on_mqtt_disconnect(self, *args):
        self.get_logger().warn('MQTT disconnected!')

    def _on_mqtt_message(self, client, userdata, msg):
        """Route incoming MQTT messages to ROS2 topics."""
        try:
            payload = msg.payload.decode().strip()

            if msg.topic == 'sensor/distance':
                distance = float(payload)

                # Float32
                f_msg = Float32()
                f_msg.data = distance
                self.distance_pub.publish(f_msg)

                # Range (standard ROS2 format)
                r_msg = Range()
                r_msg.header.stamp = self.get_clock().now().to_msg()
                r_msg.header.frame_id = 'ultrasonic_link'
                r_msg.radiation_type = Range.ULTRASOUND
                r_msg.field_of_view = 0.26  # ~15 deg for HC-SR04
                r_msg.min_range = 0.02
                r_msg.max_range = 2.0
                r_msg.range = distance
                self.range_pub.publish(r_msg)

            elif msg.topic == 'sensor/encoder':
                e_msg = String()
                e_msg.data = payload
                self.encoder_pub.publish(e_msg)

        except Exception as e:
            self.get_logger().debug(f'MQTT parse error: {e}')

    # ── ROS2 -> MQTT Callbacks ──────────────────────────────────────

    def _motor_left_cb(self, msg):
        """Forward left motor PWM to ESP32."""
        self.mqtt_client.publish('motor/left', str(msg.data))

    def _motor_right_cb(self, msg):
        """Forward right motor PWM to ESP32."""
        self.mqtt_client.publish('motor/right', str(msg.data))

    def _led_cmd_cb(self, msg):
        """Forward LED command to ESP32."""
        self.mqtt_client.publish('sensor/led', msg.data)

    # ── Cleanup ─────────────────────────────────────────────────────

    def destroy_node(self):
        """Stop motors and disconnect MQTT."""
        self.mqtt_client.publish('motor/left', '0')
        self.mqtt_client.publish('motor/right', '0')
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MqttBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
