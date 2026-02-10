"""
Distance Sensor Serial Node for ROS2 Jazzy.

Reads distance data from ESP32 over USB serial and publishes to ROS2.
This is an alternative to the MQTT version -- use if connecting via USB cable.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

try:
    import serial
except ImportError:
    raise ImportError(
        "pyserial is required. Install with: pip3 install pyserial"
    )


class DistanceSensorSerialNode(Node):
    """ROS2 node that reads distance from ESP32 via serial USB."""

    def __init__(self):
        super().__init__('distance_sensor_serial')

        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 9600)
        self.declare_parameter('distance_threshold', 0.5)

        # Read parameters
        port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.distance_threshold = self.get_parameter(
            'distance_threshold'
        ).get_parameter_value().double_value

        # Open serial connection
        try:
            self.ser = serial.Serial(port, baud, timeout=1)
            self.get_logger().info(f'Serial connected on {port} at {baud} baud')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            raise

        # ROS2 Publisher
        self.publisher_ = self.create_publisher(Float32, 'distance', 10)

        # LED state
        self.led_on = False

        # Timer to read serial data at ~10 Hz
        self.create_timer(0.1, self._read_serial)

    def _read_serial(self):
        """Read and process serial data from ESP32."""
        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()

            if line:
                try:
                    distance = float(line)

                    msg = Float32()
                    msg.data = distance
                    self.publisher_.publish(msg)

                    # LED control via serial
                    if distance < self.distance_threshold and not self.led_on:
                        self.ser.write(b'LED_ON\n')
                        self.led_on = True
                        self.get_logger().info(
                            f'Distance: {distance} - LED ON!'
                        )
                    elif distance >= self.distance_threshold and self.led_on:
                        self.ser.write(b'LED_OFF\n')
                        self.led_on = False
                        self.get_logger().info(
                            f'Distance: {distance} - LED OFF'
                        )
                    else:
                        self.get_logger().debug(f'Distance: {distance}')

                except ValueError:
                    pass

    def destroy_node(self):
        """Close serial on shutdown."""
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DistanceSensorSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
