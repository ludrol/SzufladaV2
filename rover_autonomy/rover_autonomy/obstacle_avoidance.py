"""
Obstacle Avoidance Node for ROS2 Jazzy.

Simple ultrasonic-based reactive behavior:
    - Drives forward at cruise speed
    - When ultrasonic sensor detects obstacle < threshold -> REVERSE
    - Keeps reversing until the obstacle is farther than threshold
    - Then resumes driving forward

Also supports /scan (LaserScan) from depth camera if available.

Publishes /cmd_vel.
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Range


class ObstacleAvoidanceNode(Node):
    """Reactive obstacle avoidance -- backs away from obstacles."""

    def __init__(self):
        super().__init__('obstacle_avoidance')

        # Parameters
        self.declare_parameter('ultrasonic_threshold', 0.30)
        self.declare_parameter('cruise_speed', 0.15)
        self.declare_parameter('reverse_speed', 0.12)
        self.declare_parameter('turn_speed', 0.5)
        self.declare_parameter('scan_obstacle_dist', 0.40)
        self.declare_parameter('scan_angle_front', 30.0)

        self.ultra_threshold = self.get_parameter(
            'ultrasonic_threshold').get_parameter_value().double_value
        self.cruise_speed = self.get_parameter(
            'cruise_speed').get_parameter_value().double_value
        self.reverse_speed = self.get_parameter(
            'reverse_speed').get_parameter_value().double_value
        self.turn_speed = self.get_parameter(
            'turn_speed').get_parameter_value().double_value
        self.scan_obstacle_dist = self.get_parameter(
            'scan_obstacle_dist').get_parameter_value().double_value
        self.scan_angle_front = self.get_parameter(
            'scan_angle_front').get_parameter_value().double_value

        # State
        self.ultrasonic_distance = float('inf')
        self.ultrasonic_received = False
        self.range_msg_count = 0

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscribers
        self.create_subscription(Range, 'range', self._range_callback, 10)
        self.create_subscription(LaserScan, 'scan', self._scan_callback, 10)

        # Timer for control loop at 10 Hz
        self.latest_scan = None
        self.create_timer(0.1, self._control_loop)

        self.get_logger().info(
            f'Obstacle avoidance active: '
            f'threshold={self.ultra_threshold}m, '
            f'cruise={self.cruise_speed}m/s, '
            f'reverse={self.reverse_speed}m/s')
        self.get_logger().info('Waiting for ultrasonic data on /range ...')

    def _range_callback(self, msg):
        """Process ultrasonic range data."""
        self.ultrasonic_distance = msg.range
        self.range_msg_count += 1

        if not self.ultrasonic_received:
            self.ultrasonic_received = True
            self.get_logger().info(
                f'Ultrasonic data received! Distance: {msg.range:.3f}m')

        # Log every ~50 messages (~5 seconds at 10Hz)
        if self.range_msg_count % 50 == 0:
            self.get_logger().info(
                f'Ultrasonic: {self.ultrasonic_distance:.3f}m')

    def _scan_callback(self, msg):
        """Store latest scan data."""
        self.latest_scan = msg

    def _control_loop(self):
        """Main control logic: forward when clear, reverse when blocked."""
        twist = Twist()

        # ── Ultrasonic check (highest priority) ────────────────────
        if self.ultrasonic_received:
            if self.ultrasonic_distance < self.ultra_threshold:
                # TOO CLOSE -- reverse!
                twist.linear.x = -self.reverse_speed
                twist.angular.z = 0.0
                self.get_logger().info(
                    f'REVERSING! Obstacle at {self.ultrasonic_distance:.3f}m '
                    f'(threshold: {self.ultra_threshold}m)')
                self.cmd_vel_pub.publish(twist)
                return

        # ── Laser scan check (if camera is running) ────────────────
        if self.latest_scan is not None:
            scan = self.latest_scan
            if len(scan.ranges) > 0:
                front_angle_rad = math.radians(self.scan_angle_front)
                regions = self._get_regions(scan, front_angle_rad)

                if regions['front'] < self.scan_obstacle_dist:
                    # Obstacle in front from laser -- turn away
                    twist.linear.x = 0.0
                    if regions['left'] > regions['right']:
                        twist.angular.z = self.turn_speed
                    else:
                        twist.angular.z = -self.turn_speed
                    self.cmd_vel_pub.publish(twist)
                    return

        # ── All clear -- drive forward ─────────────────────────────
        twist.linear.x = self.cruise_speed
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

    def _get_regions(self, scan, front_angle_rad):
        """Split laser scan into left, front, right regions."""
        left_ranges = []
        front_ranges = []
        right_ranges = []

        for i in range(len(scan.ranges)):
            angle = scan.angle_min + i * scan.angle_increment
            r = scan.ranges[i]

            if (r < scan.range_min or r > scan.range_max
                    or math.isinf(r) or math.isnan(r)):
                continue

            if -front_angle_rad <= angle <= front_angle_rad:
                front_ranges.append(r)
            elif angle > front_angle_rad:
                left_ranges.append(r)
            elif angle < -front_angle_rad:
                right_ranges.append(r)

        return {
            'front': min(front_ranges) if front_ranges else float('inf'),
            'left': min(left_ranges) if left_ranges else float('inf'),
            'right': min(right_ranges) if right_ranges else float('inf'),
        }

    def destroy_node(self):
        """Stop the rover on shutdown."""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
