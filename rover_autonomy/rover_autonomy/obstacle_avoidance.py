"""
Obstacle Avoidance Node for ROS2 Jazzy.

Reactive obstacle avoidance using:
    - /scan (LaserScan from depthimage_to_laserscan / depth camera)
    - /range (Range from ultrasonic sensor)

Publishes /cmd_vel to drive the rover while avoiding obstacles.

Strategy:
    1. Drive forward at cruise speed
    2. If obstacle detected in front (scan or ultrasonic), stop
    3. Turn toward the direction with most free space
    4. Resume driving forward
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Range


class ObstacleAvoidanceNode(Node):
    """Reactive obstacle avoidance using laser scan and ultrasonic."""

    def __init__(self):
        super().__init__('obstacle_avoidance')

        # Parameters
        self.declare_parameter('obstacle_distance', 0.40)
        self.declare_parameter('ultrasonic_threshold', 0.30)
        self.declare_parameter('cruise_speed', 0.15)
        self.declare_parameter('turn_speed', 0.5)
        self.declare_parameter('scan_angle_front', 30.0)

        self.obstacle_dist = self.get_parameter('obstacle_distance').get_parameter_value().double_value
        self.ultra_threshold = self.get_parameter('ultrasonic_threshold').get_parameter_value().double_value
        self.cruise_speed = self.get_parameter('cruise_speed').get_parameter_value().double_value
        self.turn_speed = self.get_parameter('turn_speed').get_parameter_value().double_value
        self.scan_angle_front = self.get_parameter('scan_angle_front').get_parameter_value().double_value

        # State
        self.ultrasonic_blocked = False
        self.enabled = True

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscribers
        self.create_subscription(LaserScan, 'scan', self._scan_callback, 10)
        self.create_subscription(Range, 'range', self._range_callback, 10)

        # Timer for control loop at 10 Hz
        self.latest_scan = None
        self.create_timer(0.1, self._control_loop)

        self.get_logger().info(
            f'Obstacle avoidance active: obstacle_dist={self.obstacle_dist}m, '
            f'cruise={self.cruise_speed}m/s'
        )

    def _range_callback(self, msg):
        """Process ultrasonic range data."""
        if msg.range < self.ultra_threshold and msg.range > msg.min_range:
            self.ultrasonic_blocked = True
        else:
            self.ultrasonic_blocked = False

    def _scan_callback(self, msg):
        """Store latest scan data."""
        self.latest_scan = msg

    def _control_loop(self):
        """Main obstacle avoidance logic."""
        if not self.enabled:
            return

        twist = Twist()

        # Check ultrasonic first (most reliable close-range)
        if self.ultrasonic_blocked:
            self.get_logger().info('Ultrasonic: obstacle detected! Turning...')
            twist.linear.x = 0.0
            twist.angular.z = self.turn_speed
            self.cmd_vel_pub.publish(twist)
            return

        # Check laser scan
        if self.latest_scan is not None:
            scan = self.latest_scan
            num_ranges = len(scan.ranges)
            if num_ranges == 0:
                return

            # Divide scan into left, front, right regions
            front_angle_rad = math.radians(self.scan_angle_front)
            regions = self._get_regions(scan, front_angle_rad)

            front_min = regions['front']
            left_min = regions['left']
            right_min = regions['right']

            if front_min < self.obstacle_dist:
                # Obstacle in front -- turn toward the side with more space
                twist.linear.x = 0.0
                if left_min > right_min:
                    twist.angular.z = self.turn_speed
                    self.get_logger().debug('Obstacle front, turning left')
                else:
                    twist.angular.z = -self.turn_speed
                    self.get_logger().debug('Obstacle front, turning right')
            else:
                # Path is clear -- drive forward
                twist.linear.x = self.cruise_speed
                twist.angular.z = 0.0
        else:
            # No scan data yet -- drive slowly
            twist.linear.x = self.cruise_speed * 0.5
            twist.angular.z = 0.0

        self.cmd_vel_pub.publish(twist)

    def _get_regions(self, scan, front_angle_rad):
        """Split laser scan into left, front, right regions and get minimums."""
        num = len(scan.ranges)
        angle_min = scan.angle_min
        angle_increment = scan.angle_increment

        left_ranges = []
        front_ranges = []
        right_ranges = []

        for i in range(num):
            angle = angle_min + i * angle_increment
            r = scan.ranges[i]

            # Skip invalid readings
            if r < scan.range_min or r > scan.range_max or math.isinf(r) or math.isnan(r):
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
