"""
Rover Teleop Node for ROS2 Jazzy.

Provides keyboard-based teleoperation of the ESP32 rover via MQTT.
Subscribes to MQTT distance/encoder data and publishes to ROS2 topics.
Sends motor and LED commands to ESP32 via MQTT.

Controls:
    W = Forward       A = Turn Left
    S = Backward      D = Turn Right
    SPACE = Stop
    L = Toggle LED
    +/- = Increase/Decrease speed
    Q = Quit
"""

import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Twist

try:
    import paho.mqtt.client as mqtt
    PAHO_V2 = hasattr(mqtt, 'CallbackAPIVersion')
except ImportError:
    raise ImportError(
        "paho-mqtt is required. Install with: pip3 install paho-mqtt"
    )


class RoverTeleopNode(Node):
    """Keyboard teleoperation node for the rover."""

    def __init__(self):
        super().__init__('rover_teleop')

        # Declare parameters
        self.declare_parameter('mqtt_broker', '192.168.1.1')
        self.declare_parameter('mqtt_port', 1883)
        self.declare_parameter('default_speed', 150)

        # Read parameters
        self.mqtt_broker = self.get_parameter('mqtt_broker').get_parameter_value().string_value
        self.mqtt_port = self.get_parameter('mqtt_port').get_parameter_value().integer_value
        self.motor_speed = self.get_parameter('default_speed').get_parameter_value().integer_value

        # MQTT Topics (matching ESP32 code)
        self.topic_led = 'sensor/led'
        self.topic_motor_left = 'motor/left'
        self.topic_motor_right = 'motor/right'
        self.topic_distance = 'sensor/distance'
        self.topic_encoder = 'sensor/encoder'

        # State
        self.led_on = False
        self.current_left = 0
        self.current_right = 0

        # ROS2 Publishers - republish sensor data from MQTT into ROS2
        self.distance_pub = self.create_publisher(Float32, 'distance', 10)
        self.encoder_pub = self.create_publisher(String, 'encoder', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # --- MQTT Client Setup (paho-mqtt v2.x compatible) ---
        if PAHO_V2:
            self.mqtt_client = mqtt.Client(
                callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
                client_id='ros2_rover_teleop'
            )
        else:
            self.mqtt_client = mqtt.Client(client_id='ros2_rover_teleop')

        self.mqtt_client.on_connect = self._on_mqtt_connect
        self.mqtt_client.on_message = self._on_mqtt_message

        try:
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, 60)
            self.mqtt_client.loop_start()
            self.get_logger().info(
                f'Connected to MQTT broker at {self.mqtt_broker}'
            )
        except Exception as e:
            self.get_logger().error(f'Failed to connect: {e}')
            return

        self._print_instructions()
        self._read_keyboard()

    def _on_mqtt_connect(self, client, userdata, flags, reason_code, properties=None):
        """MQTT connect callback (v2 API)."""
        self.get_logger().info('MQTT Connected!')
        client.subscribe(self.topic_distance)
        client.subscribe(self.topic_encoder)

    def _on_mqtt_message(self, client, userdata, msg):
        """MQTT message callback - bridge sensor data to ROS2."""
        try:
            if msg.topic == self.topic_distance:
                distance = float(msg.payload.decode())
                ros_msg = Float32()
                ros_msg.data = distance
                self.distance_pub.publish(ros_msg)
            elif msg.topic == self.topic_encoder:
                enc_msg = String()
                enc_msg.data = msg.payload.decode()
                self.encoder_pub.publish(enc_msg)
        except Exception:
            pass

    def _print_instructions(self):
        """Print teleop control instructions."""
        print('\n' + '=' * 50)
        print('    MARS ROVER TELEOP CONTROL (ROS2 Jazzy)')
        print('=' * 50)
        print('  Movement:')
        print('            W')
        print('        A   S   D       W = Forward')
        print('                        S = Backward')
        print('                        A = Turn Left')
        print('                        D = Turn Right')
        print('')
        print('  Other Controls:')
        print('    SPACE = Stop')
        print('    L     = Toggle LED')
        print('    +/-   = Increase/Decrease speed')
        print('    Q     = Quit')
        print('')
        print(f'  Current Speed: {self.motor_speed}/255')
        print('=' * 50 + '\n')

    def _get_key(self):
        """Read a single keypress from terminal."""
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

            if key.lower() == 'w':
                self._move_forward()
            elif key.lower() == 's':
                self._move_backward()
            elif key.lower() == 'a':
                self._turn_left()
            elif key.lower() == 'd':
                self._turn_right()
            elif key == ' ':
                self._stop()
            elif key.lower() == 'l':
                self._toggle_led()
            elif key in ('+', '='):
                self._increase_speed()
            elif key == '-':
                self._decrease_speed()
            elif key.lower() == 'q' or key == '\x03':
                self._stop()
                self.get_logger().info('Exiting...')
                break

    def _move_forward(self):
        """Drive forward (both motors negative per original calibration)."""
        self.current_left = -self.motor_speed
        self.current_right = -self.motor_speed
        self._send_motor_commands()
        self._publish_cmd_vel(1.0, 0.0)
        self.get_logger().info(f'FORWARD (speed: {self.motor_speed})')

    def _move_backward(self):
        """Drive backward (both motors positive per original calibration)."""
        self.current_left = self.motor_speed
        self.current_right = self.motor_speed
        self._send_motor_commands()
        self._publish_cmd_vel(-1.0, 0.0)
        self.get_logger().info(f'BACKWARD (speed: {self.motor_speed})')

    def _turn_left(self):
        """Turn left (differential drive)."""
        self.current_left = self.motor_speed
        self.current_right = -self.motor_speed
        self._send_motor_commands()
        self._publish_cmd_vel(0.0, 1.0)
        self.get_logger().info(f'TURN LEFT (speed: {self.motor_speed})')

    def _turn_right(self):
        """Turn right (differential drive)."""
        self.current_left = -self.motor_speed
        self.current_right = self.motor_speed
        self._send_motor_commands()
        self._publish_cmd_vel(0.0, -1.0)
        self.get_logger().info(f'TURN RIGHT (speed: {self.motor_speed})')

    def _stop(self):
        """Stop both motors."""
        self.current_left = 0
        self.current_right = 0
        self._send_motor_commands()
        self._publish_cmd_vel(0.0, 0.0)
        self.get_logger().info('STOP')

    def _send_motor_commands(self):
        """Publish motor speed commands via MQTT to ESP32."""
        self.mqtt_client.publish(self.topic_motor_left, str(self.current_left))
        self.mqtt_client.publish(self.topic_motor_right, str(self.current_right))

    def _publish_cmd_vel(self, linear_x, angular_z):
        """Also publish a Twist message on cmd_vel for ROS2 ecosystem."""
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_vel_pub.publish(twist)

    def _toggle_led(self):
        """Toggle ESP32 LED via MQTT."""
        self.led_on = not self.led_on
        command = 'ON' if self.led_on else 'OFF'
        self.mqtt_client.publish(self.topic_led, command)
        self.get_logger().info(f'LED: {command}')

    def _increase_speed(self):
        """Increase motor speed by 25 (max 255)."""
        self.motor_speed = min(255, self.motor_speed + 25)
        self.get_logger().info(f'Speed: {self.motor_speed}/255')

    def _decrease_speed(self):
        """Decrease motor speed by 25 (min 25)."""
        self.motor_speed = max(25, self.motor_speed - 25)
        self.get_logger().info(f'Speed: {self.motor_speed}/255')

    def destroy_node(self):
        """Cleanup MQTT on shutdown."""
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RoverTeleopNode()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
