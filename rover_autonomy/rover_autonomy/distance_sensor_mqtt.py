"""
Distance Sensor MQTT Node for ROS2 Jazzy.

Bridges the ESP32 ultrasonic distance sensor data from MQTT to ROS2.
Subscribes to 'sensor/distance' MQTT topic and publishes on ROS2 'distance' topic.
Also controls the LED based on distance threshold.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Range

try:
    import paho.mqtt.client as mqtt
    PAHO_V2 = hasattr(mqtt, 'CallbackAPIVersion')
except ImportError:
    raise ImportError(
        "paho-mqtt is required. Install with: pip3 install paho-mqtt"
    )


class DistanceSensorMQTTNode(Node):
    """ROS2 node that bridges MQTT distance sensor data to ROS2 topics."""

    def __init__(self):
        super().__init__('distance_sensor_mqtt')

        # Declare parameters with defaults (configurable at launch)
        self.declare_parameter('mqtt_broker', '192.168.1.1')
        self.declare_parameter('mqtt_port', 1883)
        self.declare_parameter('distance_threshold', 0.5)

        # Read parameters
        self.mqtt_broker = self.get_parameter('mqtt_broker').get_parameter_value().string_value
        self.mqtt_port = self.get_parameter('mqtt_port').get_parameter_value().integer_value
        self.distance_threshold = self.get_parameter(
            'distance_threshold'
        ).get_parameter_value().double_value

        # MQTT Topics (matching ESP32 code)
        self.mqtt_topic_distance = 'sensor/distance'
        self.mqtt_topic_led = 'sensor/led'

        # ROS2 Publishers
        self.distance_pub = self.create_publisher(Float32, 'distance', 10)
        self.range_pub = self.create_publisher(Range, 'range', 10)

        # LED state tracking
        self.led_on = False

        # --- MQTT Client Setup (paho-mqtt v2.x compatible) ---
        if PAHO_V2:
            self.mqtt_client = mqtt.Client(
                callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
                client_id='ros2_distance_sensor'
            )
        else:
            self.mqtt_client = mqtt.Client(client_id='ros2_distance_sensor')

        self.mqtt_client.on_connect = self._on_mqtt_connect
        self.mqtt_client.on_message = self._on_mqtt_message
        self.mqtt_client.on_disconnect = self._on_mqtt_disconnect

        # Connect to MQTT broker
        self._connect_mqtt()

    def _connect_mqtt(self):
        """Establish connection to MQTT broker."""
        try:
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, 60)
            self.mqtt_client.loop_start()
            self.get_logger().info(
                f'Connecting to MQTT broker at {self.mqtt_broker}:{self.mqtt_port}'
            )
        except Exception as e:
            self.get_logger().error(f'Failed to connect to MQTT broker: {e}')

    def _on_mqtt_connect(self, client, userdata, flags, reason_code, properties=None):
        """Called when connected to MQTT broker (paho v2 signature)."""
        self.get_logger().info(f'MQTT Connected (reason: {reason_code})')
        client.subscribe(self.mqtt_topic_distance)
        self.get_logger().info(f'Subscribed to {self.mqtt_topic_distance}')

    def _on_mqtt_disconnect(self, *args):
        """Called when disconnected from MQTT broker (compatible with paho v1 and v2)."""
        self.get_logger().warn('MQTT Disconnected')

    def _on_mqtt_message(self, client, userdata, msg):
        """Called when an MQTT message is received."""
        try:
            distance = float(msg.payload.decode())

            # Publish Float32 on 'distance' topic
            float_msg = Float32()
            float_msg.data = distance
            self.distance_pub.publish(float_msg)

            # Publish Range message (more standard ROS2 format)
            range_msg = Range()
            range_msg.header.stamp = self.get_clock().now().to_msg()
            range_msg.header.frame_id = 'ultrasonic_sensor'
            range_msg.radiation_type = Range.ULTRASOUND
            range_msg.field_of_view = 0.26  # ~15 degrees for HC-SR04
            range_msg.min_range = 0.02  # 2cm
            range_msg.max_range = 2.0   # 200cm
            range_msg.range = distance
            self.range_pub.publish(range_msg)

            # --- LED Control Logic ---
            if distance < self.distance_threshold and not self.led_on:
                self.mqtt_client.publish(self.mqtt_topic_led, 'ON')
                self.led_on = True
                self.get_logger().info(
                    f'Distance: {distance:.2f}m - LED ON!'
                )
            elif distance >= self.distance_threshold and self.led_on:
                self.mqtt_client.publish(self.mqtt_topic_led, 'OFF')
                self.led_on = False
                self.get_logger().info(
                    f'Distance: {distance:.2f}m - LED OFF'
                )
            else:
                self.get_logger().debug(f'Distance: {distance:.2f}m')

        except ValueError:
            self.get_logger().warn(
                f'Invalid distance value: {msg.payload}'
            )

    def destroy_node(self):
        """Cleanup MQTT on shutdown."""
        self.get_logger().info('Shutting down distance sensor node...')
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DistanceSensorMQTTNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
