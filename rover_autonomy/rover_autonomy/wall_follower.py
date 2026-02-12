"""
Wall Follower Node for ROS2 Jazzy.

Follows a wall at a set distance using a simple PID controller.
Uses /scan (LaserScan) data to maintain distance from the nearest wall.

Publishes /cmd_vel.

Parameters:
    desired_distance: target distance from wall (m)
    side: 'left' or 'right' wall to follow
    cruise_speed: forward speed (m/s)
    kp, ki, kd: PID gains
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class WallFollowerNode(Node):
    """PID-based wall following using laser scan."""

    def __init__(self):
        super().__init__('wall_follower')

        # Parameters
        self.declare_parameter('desired_distance', 0.35)
        self.declare_parameter('side', 'right')
        self.declare_parameter('cruise_speed', 0.12)
        self.declare_parameter('kp', 2.0)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.5)
        self.declare_parameter('front_obstacle_dist', 0.35)

        self.desired_dist = self.get_parameter('desired_distance').get_parameter_value().double_value
        self.side = self.get_parameter('side').get_parameter_value().string_value
        self.cruise_speed = self.get_parameter('cruise_speed').get_parameter_value().double_value
        self.kp = self.get_parameter('kp').get_parameter_value().double_value
        self.ki = self.get_parameter('ki').get_parameter_value().double_value
        self.kd = self.get_parameter('kd').get_parameter_value().double_value
        self.front_obstacle_dist = self.get_parameter('front_obstacle_dist').get_parameter_value().double_value

        # PID state
        self.integral = 0.0
        self.prev_error = 0.0
        self.last_time = self.get_clock().now()

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscribers
        self.create_subscription(LaserScan, 'scan', self._scan_callback, 10)

        self.get_logger().info(
            f'Wall follower active: side={self.side}, '
            f'desired_dist={self.desired_dist}m, speed={self.cruise_speed}m/s'
        )

    def _scan_callback(self, msg):
        """Process laser scan and apply PID wall following."""
        num = len(msg.ranges)
        if num == 0:
            return

        # Get side distance and front distance
        side_dist = self._get_side_distance(msg)
        front_dist = self._get_front_distance(msg)

        # PID control
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        if dt <= 0.0:
            return

        error = self.desired_dist - side_dist

        self.integral += error * dt
        # Anti-windup
        self.integral = max(-1.0, min(1.0, self.integral))

        derivative = (error - self.prev_error) / dt
        self.prev_error = error

        angular_correction = self.kp * error + self.ki * self.integral + self.kd * derivative

        # If following right wall, positive error means too close -> turn left (positive angular)
        # If following left wall, positive error means too close -> turn right (negative angular)
        if self.side == 'left':
            angular_correction = -angular_correction

        twist = Twist()

        # Check for front obstacle
        if front_dist < self.front_obstacle_dist:
            # Turn away from wall
            twist.linear.x = 0.0
            if self.side == 'right':
                twist.angular.z = 0.8  # Turn left
            else:
                twist.angular.z = -0.8  # Turn right
            self.get_logger().debug('Front obstacle, turning')
        else:
            twist.linear.x = self.cruise_speed
            twist.angular.z = max(-1.0, min(1.0, angular_correction))

        self.cmd_vel_pub.publish(twist)

    def _get_side_distance(self, scan):
        """Get minimum distance on the desired side."""
        num = len(scan.ranges)
        side_ranges = []

        for i in range(num):
            angle = scan.angle_min + i * scan.angle_increment
            r = scan.ranges[i]

            if r < scan.range_min or r > scan.range_max or math.isinf(r) or math.isnan(r):
                continue

            # Right side: angles between -90 and -30 degrees
            # Left side: angles between 30 and 90 degrees
            if self.side == 'right' and -math.pi / 2 <= angle <= -math.pi / 6:
                side_ranges.append(r)
            elif self.side == 'left' and math.pi / 6 <= angle <= math.pi / 2:
                side_ranges.append(r)

        return min(side_ranges) if side_ranges else float('inf')

    def _get_front_distance(self, scan):
        """Get minimum distance in front."""
        num = len(scan.ranges)
        front_ranges = []
        front_angle = math.radians(20.0)

        for i in range(num):
            angle = scan.angle_min + i * scan.angle_increment
            r = scan.ranges[i]

            if r < scan.range_min or r > scan.range_max or math.isinf(r) or math.isnan(r):
                continue

            if -front_angle <= angle <= front_angle:
                front_ranges.append(r)

        return min(front_ranges) if front_ranges else float('inf')

    def destroy_node(self):
        """Stop on shutdown."""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WallFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
