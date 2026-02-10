"""
Keyboard LED Control Node for ROS2 Jazzy.

Simple node to toggle the ESP32 LED via keyboard and MQTT.

Controls:
    L = Toggle LED
    Q = Quit
"""

import sys
import termios
import tty

import rclpy
from rclpy.node import Node

try:
    import paho.mqtt.client as mqtt
    PAHO_V2 = hasattr(mqtt, 'CallbackAPIVersion')
except ImportError:
    raise ImportError(
        "paho-mqtt is required. Install with: pip3 install paho-mqtt"
    )


class KeyboardLEDNode(Node):
    """Simple keyboard LED toggle node."""

    def __init__(self):
        super().__init__('keyboard_led')

        # Declare parameters
        self.declare_parameter('mqtt_broker', '192.168.1.1')
        self.declare_parameter('mqtt_port', 1883)

        # Read parameters
        self.mqtt_broker = self.get_parameter('mqtt_broker').get_parameter_value().string_value
        self.mqtt_port = self.get_parameter('mqtt_port').get_parameter_value().integer_value

        self.mqtt_topic_led = 'sensor/led'
        self.led_on = False

        # --- MQTT Client Setup (paho-mqtt v2.x compatible) ---
        if PAHO_V2:
            self.mqtt_client = mqtt.Client(
                callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
                client_id='ros2_keyboard_led'
            )
        else:
            self.mqtt_client = mqtt.Client(client_id='ros2_keyboard_led')

        try:
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, 60)
            self.mqtt_client.loop_start()
            self.get_logger().info(
                f'Connected to MQTT broker at {self.mqtt_broker}'
            )
        except Exception as e:
            self.get_logger().error(f'Failed to connect: {e}')
            return

        self.get_logger().info('=' * 45)
        self.get_logger().info('  KEYBOARD LED CONTROL (ROS2 Jazzy)')
        self.get_logger().info('=' * 45)
        self.get_logger().info("  Press 'L' to toggle LED ON/OFF")
        self.get_logger().info("  Press 'Q' to quit")
        self.get_logger().info('=' * 45)

        self._read_keyboard()

    def _get_key(self):
        """Read a single keypress."""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def _read_keyboard(self):
        """Main keyboard reading loop."""
        while rclpy.ok():
            key = self._get_key()

            if key.lower() == 'l':
                self._toggle_led()
            elif key.lower() == 'q' or key == '\x03':
                self.get_logger().info('Exiting...')
                break

    def _toggle_led(self):
        """Toggle LED state via MQTT."""
        self.led_on = not self.led_on
        command = 'ON' if self.led_on else 'OFF'
        self.mqtt_client.publish(self.mqtt_topic_led, command)
        self.get_logger().info(f'LED toggled: {command}')

    def destroy_node(self):
        """Cleanup MQTT on shutdown."""
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardLEDNode()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
