"""
Encoder Calibration Tool for ROS2 Jazzy.

Runs ONE motor at slow speed for a set number of encoder ticks, then stops.
Use this to determine how many ticks = one full wheel revolution.

Usage:
    ros2 run rover_autonomy encoder_calibration --ros-args -p ticks:=20
    ros2 run rover_autonomy encoder_calibration --ros-args -p ticks:=40
    ros2 run rover_autonomy encoder_calibration --ros-args -p ticks:=100

Watch the wheel: when the mark lines up after one full spin,
the ticks value you used is your encoder_ticks_per_rev.
"""

import rclpy
from rclpy.node import Node

try:
    import paho.mqtt.client as mqtt
    PAHO_V2 = hasattr(mqtt, 'CallbackAPIVersion')
except ImportError:
    raise ImportError("paho-mqtt is required.")


class EncoderCalibrationNode(Node):
    """Run a motor for N ticks to calibrate encoder."""

    def __init__(self):
        super().__init__('encoder_calibration')

        self.declare_parameter('mqtt_broker', '192.168.1.1')
        self.declare_parameter('mqtt_port', 1883)
        self.declare_parameter('ticks', 20)
        self.declare_parameter('speed', 100)
        self.declare_parameter('motor', 'left')

        self.mqtt_broker = self.get_parameter('mqtt_broker').get_parameter_value().string_value
        self.mqtt_port = self.get_parameter('mqtt_port').get_parameter_value().integer_value
        self.target_ticks = self.get_parameter('ticks').get_parameter_value().integer_value
        self.speed = self.get_parameter('speed').get_parameter_value().integer_value
        self.motor = self.get_parameter('motor').get_parameter_value().string_value

        self.start_enc1 = None
        self.start_enc2 = None
        self.running = False
        self.done = False

        # MQTT
        if PAHO_V2:
            self.mqtt_client = mqtt.Client(
                callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
                client_id='ros2_encoder_cal'
            )
        else:
            self.mqtt_client = mqtt.Client(client_id='ros2_encoder_cal')

        self.mqtt_client.on_connect = self._on_connect
        self.mqtt_client.on_message = self._on_message

        try:
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, 60)
            self.mqtt_client.loop_start()
        except Exception as e:
            self.get_logger().error(f'MQTT connect failed: {e}')
            return

        self.get_logger().info('')
        self.get_logger().info('=' * 50)
        self.get_logger().info('  ENCODER CALIBRATION')
        self.get_logger().info('=' * 50)
        self.get_logger().info(f'  Motor: {self.motor}')
        self.get_logger().info(f'  Target ticks: {self.target_ticks}')
        self.get_logger().info(f'  Speed: {self.speed}')
        self.get_logger().info('  Waiting for first encoder reading...')
        self.get_logger().info('=' * 50)

    def _on_connect(self, client, userdata, flags, reason_code, properties=None):
        client.subscribe('sensor/encoder')

    def _on_message(self, client, userdata, msg):
        if self.done:
            return

        try:
            parts = msg.payload.decode().strip().split(',')
            if len(parts) != 4:
                return

            enc1 = int(parts[0])
            enc2 = int(parts[1])

            # First reading -- store baseline and start motor
            if self.start_enc1 is None:
                self.start_enc1 = enc1
                self.start_enc2 = enc2
                self.running = True

                # Start the motor
                pwm = -self.speed  # negative = forward on this rover
                if self.motor == 'left':
                    self.mqtt_client.publish('motor/left', str(pwm))
                    self.mqtt_client.publish('motor/right', '0')
                else:
                    self.mqtt_client.publish('motor/left', '0')
                    self.mqtt_client.publish('motor/right', str(pwm))

                self.get_logger().info(f'  Started! Baseline: enc1={enc1}, enc2={enc2}')
                return

            # Track progress
            if self.motor == 'left':
                traveled = enc1 - self.start_enc1
            else:
                traveled = enc2 - self.start_enc2

            self.get_logger().info(f'  Ticks: {traveled} / {self.target_ticks}')

            # Check if target reached
            if traveled >= self.target_ticks:
                # Stop motor
                self.mqtt_client.publish('motor/left', '0')
                self.mqtt_client.publish('motor/right', '0')
                self.done = True
                self.running = False

                self.get_logger().info('')
                self.get_logger().info('=' * 50)
                self.get_logger().info(f'  DONE! Traveled {traveled} ticks.')
                self.get_logger().info('  Did the wheel complete exactly one full turn?')
                self.get_logger().info('  - YES -> encoder_ticks_per_rev = ' + str(self.target_ticks))
                self.get_logger().info('  - NO (not enough) -> try higher ticks value')
                self.get_logger().info('  - NO (too much) -> try lower ticks value')
                self.get_logger().info('=' * 50)

        except Exception as e:
            self.get_logger().debug(f'Parse error: {e}')

    def destroy_node(self):
        # Safety: stop motors
        self.mqtt_client.publish('motor/left', '0')
        self.mqtt_client.publish('motor/right', '0')
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EncoderCalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
