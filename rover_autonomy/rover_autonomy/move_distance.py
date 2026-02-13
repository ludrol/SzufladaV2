"""
Move Distance Node for ROS2 Jazzy.

Drives the rover a specified distance (meters) or rotates a specified
angle (degrees), using ODOMETRY feedback for accurate distance tracking.

The node subscribes to /odom and accumulates the incremental path length
(odometer-style), so even if the rover curves slightly the reported
distance is correct.

Usage:
    # Move forward 0.5 meters:
    ros2 run rover_autonomy move_distance --ros-args -p distance:=0.5

    # Move backward 0.3 meters:
    ros2 run rover_autonomy move_distance --ros-args -p distance:=-0.3

    # Rotate 90 degrees left:
    ros2 run rover_autonomy move_distance --ros-args -p angle:=90.0

    # Rotate 45 degrees right:
    ros2 run rover_autonomy move_distance --ros-args -p angle:=-45.0

    # Move 1 meter forward then rotate 90 degrees:
    ros2 run rover_autonomy move_distance --ros-args -p distance:=1.0 -p angle:=90.0

    # Custom speed (cmd_vel value, not PWM):
    ros2 run rover_autonomy move_distance --ros-args -p distance:=1.0 -p linear_speed:=0.15
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def yaw_from_quaternion(q):
    """Extract yaw from a quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(a):
    """Normalize angle to [-pi, pi]."""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class MoveDistanceNode(Node):
    """Drive a set distance/angle using odometry feedback."""

    def __init__(self):
        super().__init__('move_distance')

        # Parameters
        self.declare_parameter('distance', 0.0)         # meters (+ forward, - backward)
        self.declare_parameter('angle', 0.0)             # degrees (+ left, - right)
        self.declare_parameter('linear_speed', 0.15)     # cmd_vel linear.x magnitude
        self.declare_parameter('angular_speed', 0.85)    # cmd_vel angular.z magnitude
        self.declare_parameter('distance_tolerance', 0.02)  # stop within 2cm
        self.declare_parameter('angle_tolerance', 20.0)     # stop within 20 degrees (caster wheels cause overshoot)

        self.target_distance = self.get_parameter('distance').get_parameter_value().double_value
        self.target_angle_deg = self.get_parameter('angle').get_parameter_value().double_value
        self.linear_speed = self.get_parameter('linear_speed').get_parameter_value().double_value
        self.angular_speed = self.get_parameter('angular_speed').get_parameter_value().double_value
        self.distance_tolerance = self.get_parameter('distance_tolerance').get_parameter_value().double_value
        self.angle_tolerance_deg = self.get_parameter('angle_tolerance').get_parameter_value().double_value

        self.target_angle_rad = math.radians(self.target_angle_deg)
        self.angle_tolerance_rad = math.radians(self.angle_tolerance_deg)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Odometry state
        self.prev_x = None
        self.prev_y = None
        self.prev_yaw = None
        self.cumulative_distance = 0.0
        self.cumulative_angle = 0.0  # radians

        # Phase machine: wait -> linear -> rotate -> done
        self.phase = 'wait'
        self.odom_received = False
        self.stop_sent_count = 0
        self.last_log_time = 0.0

        # Subscribe to odometry
        self.create_subscription(Odometry, 'odom', self._odom_callback, 50)

        # Control loop at 20Hz
        self.create_timer(0.05, self._control_loop)

        self.get_logger().info('')
        self.get_logger().info('=' * 50)
        self.get_logger().info('  MOVE DISTANCE (odometry feedback)')
        self.get_logger().info('=' * 50)
        if self.target_distance != 0.0:
            direction = 'forward' if self.target_distance > 0 else 'backward'
            self.get_logger().info(
                f'  Distance: {abs(self.target_distance):.2f} m ({direction})')
        if self.target_angle_deg != 0.0:
            direction = 'left' if self.target_angle_deg > 0 else 'right'
            self.get_logger().info(
                f'  Angle: {abs(self.target_angle_deg):.1f} deg ({direction})')
        self.get_logger().info(f'  Linear speed: {self.linear_speed:.2f} m/s')
        self.get_logger().info(f'  Angular speed: {self.angular_speed:.2f} rad/s')
        self.get_logger().info('  Waiting for odometry...')
        self.get_logger().info('=' * 50)

    def _odom_callback(self, msg):
        """Accumulate distance and angle from odometry."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)

        if self.prev_x is not None:
            # Accumulate incremental distance (odometer style)
            dx = x - self.prev_x
            dy = y - self.prev_y
            step = math.sqrt(dx * dx + dy * dy)

            # Safety: reject impossible jumps (> 0.1m per odom tick ~200ms)
            # These come from MQTT reconnections causing position glitches
            if step > 0.1:
                self.get_logger().warn(
                    f'  Odom jump rejected: {step:.3f} m '
                    f'(pos {self.prev_x:.3f},{self.prev_y:.3f} -> {x:.3f},{y:.3f})')
                # Update prev without accumulating
                self.prev_x = x
                self.prev_y = y
                self.prev_yaw = yaw
                return

            self.cumulative_distance += step

            # Accumulate incremental angle
            d_yaw = normalize_angle(yaw - self.prev_yaw)
            self.cumulative_angle += d_yaw

        self.prev_x = x
        self.prev_y = y
        self.prev_yaw = yaw

        if not self.odom_received:
            self.odom_received = True
            self.get_logger().info('  Odometry received! Starting...')

    def _control_loop(self):
        """Main control loop."""
        twist = Twist()

        if not self.odom_received:
            # Still waiting for odometry data
            return

        # Start linear phase on first odom
        if self.phase == 'wait':
            if abs(self.target_distance) > self.distance_tolerance:
                self.phase = 'linear'
                # Reset accumulators at the start of movement
                self.cumulative_distance = 0.0
                self.cumulative_angle = 0.0
                self.get_logger().info('  Phase: DRIVE')
            elif abs(self.target_angle_rad) > self.angle_tolerance_rad:
                self.phase = 'rotate'
                self.cumulative_distance = 0.0
                self.cumulative_angle = 0.0
                self.get_logger().info('  Phase: ROTATE')
            else:
                self.phase = 'done'
                self.get_logger().info('  Nothing to do.')

        if self.phase == 'done':
            if self.stop_sent_count < 20:
                self.cmd_vel_pub.publish(twist)  # send zero velocity
                self.stop_sent_count += 1
                if self.stop_sent_count == 20:
                    self.get_logger().info('  Complete!')
            return

        if self.phase == 'linear':
            traveled = self.cumulative_distance
            remaining = abs(self.target_distance) - traveled

            # Log every ~2 seconds
            now = self.get_clock().now().nanoseconds / 1e9
            if now - self.last_log_time >= 2.0:
                self.last_log_time = now
                self.get_logger().info(
                    f'  Traveled: {traveled:.3f} / {abs(self.target_distance):.2f} m  '
                    f'(remaining: {remaining:.3f} m)')

            if remaining <= self.distance_tolerance:
                # Stop
                for _ in range(5):
                    self.cmd_vel_pub.publish(twist)

                self.get_logger().info(
                    f'  Drive done! Traveled {traveled:.3f} m')

                # Move to rotate phase if needed
                if abs(self.target_angle_rad) > self.angle_tolerance_rad:
                    self.phase = 'rotate'
                    self.cumulative_angle = 0.0
                    self.get_logger().info('  Phase: ROTATE')
                else:
                    self.phase = 'done'
                return

            # Drive at fixed speed
            speed = self.linear_speed
            if self.target_distance < 0:
                speed = -speed
            twist.linear.x = speed
            self.cmd_vel_pub.publish(twist)

        elif self.phase == 'rotate':
            rotated = abs(self.cumulative_angle)
            target = abs(self.target_angle_rad)
            remaining = target - rotated

            if remaining <= self.angle_tolerance_rad:
                # Stop
                for _ in range(5):
                    self.cmd_vel_pub.publish(twist)

                self.get_logger().info(
                    f'  Rotate done! Rotated {math.degrees(rotated):.1f} deg')
                self.phase = 'done'
                return

            # Rotate at fixed speed
            speed = self.angular_speed
            if self.target_angle_deg < 0:
                speed = -speed
            twist.angular.z = speed
            self.cmd_vel_pub.publish(twist)

    def destroy_node(self):
        twist = Twist()
        for _ in range(5):
            self.cmd_vel_pub.publish(twist)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MoveDistanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
