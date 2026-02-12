"""
Visual Tracker Node for ROS2 Jazzy.

Uses the Intel RealSense D435i RGB stream to detect and track targets.
Supports color tracking (HSV-based) and can be extended for object detection.

Publishes /cmd_vel to steer the rover toward the tracked target.

Subscribes to the RealSense RGB image topic (default: /camera/camera/color/image_raw).
"""

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class VisualTrackerNode(Node):
    """Track a colored object and steer the rover toward it."""

    def __init__(self):
        super().__init__('visual_tracker')

        # Parameters
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('linear_speed', 0.12)
        self.declare_parameter('angular_gain', 0.003)
        self.declare_parameter('min_area', 500)
        self.declare_parameter('target_area', 15000)
        self.declare_parameter('deadzone', 30)

        # Color range (HSV) -- default: red object
        # Adjust these for your target color
        self.declare_parameter('hue_low', 0)
        self.declare_parameter('hue_high', 10)
        self.declare_parameter('sat_low', 120)
        self.declare_parameter('sat_high', 255)
        self.declare_parameter('val_low', 70)
        self.declare_parameter('val_high', 255)

        # Secondary red range (red wraps around in HSV)
        self.declare_parameter('hue_low2', 170)
        self.declare_parameter('hue_high2', 180)

        # Read parameters
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.linear_speed = self.get_parameter('linear_speed').get_parameter_value().double_value
        self.angular_gain = self.get_parameter('angular_gain').get_parameter_value().double_value
        self.min_area = self.get_parameter('min_area').get_parameter_value().integer_value
        self.target_area = self.get_parameter('target_area').get_parameter_value().integer_value
        self.deadzone = self.get_parameter('deadzone').get_parameter_value().integer_value

        self.hue_low = self.get_parameter('hue_low').get_parameter_value().integer_value
        self.hue_high = self.get_parameter('hue_high').get_parameter_value().integer_value
        self.sat_low = self.get_parameter('sat_low').get_parameter_value().integer_value
        self.sat_high = self.get_parameter('sat_high').get_parameter_value().integer_value
        self.val_low = self.get_parameter('val_low').get_parameter_value().integer_value
        self.val_high = self.get_parameter('val_high').get_parameter_value().integer_value
        self.hue_low2 = self.get_parameter('hue_low2').get_parameter_value().integer_value
        self.hue_high2 = self.get_parameter('hue_high2').get_parameter_value().integer_value

        self.bridge = CvBridge()

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.debug_pub = self.create_publisher(Image, 'visual_tracker/debug_image', 10)

        # Subscriber
        self.create_subscription(Image, image_topic, self._image_callback, 10)

        self.get_logger().info(
            f'Visual tracker started on {image_topic}'
        )

    def _image_callback(self, msg):
        """Process image, find target, publish cmd_vel."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CV bridge error: {e}')
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        h, w = frame.shape[:2]
        center_x = w // 2

        # Create mask for target color
        lower1 = np.array([self.hue_low, self.sat_low, self.val_low])
        upper1 = np.array([self.hue_high, self.sat_high, self.val_high])
        mask1 = cv2.inRange(hsv, lower1, upper1)

        lower2 = np.array([self.hue_low2, self.sat_low, self.val_low])
        upper2 = np.array([self.hue_high2, self.sat_high, self.val_high])
        mask2 = cv2.inRange(hsv, lower2, upper2)

        mask = mask1 | mask2

        # Clean up mask
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        twist = Twist()

        if contours:
            # Find largest contour
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)

            if area > self.min_area:
                # Get center of the target
                M = cv2.moments(largest)
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])

                    # Error from center of frame
                    error_x = cx - center_x

                    # Angular: steer toward target
                    if abs(error_x) > self.deadzone:
                        twist.angular.z = -self.angular_gain * error_x
                    else:
                        twist.angular.z = 0.0

                    # Linear: drive forward if target is far (small area),
                    # stop if close (large area)
                    if area < self.target_area:
                        twist.linear.x = self.linear_speed
                    else:
                        twist.linear.x = 0.0

                    self.get_logger().debug(
                        f'Target: cx={cx}, area={area:.0f}, '
                        f'err={error_x}, cmd=({twist.linear.x:.2f}, {twist.angular.z:.2f})'
                    )

                    # Draw debug visualization
                    cv2.drawContours(frame, [largest], -1, (0, 255, 0), 2)
                    cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
                    cv2.line(frame, (center_x, 0), (center_x, h), (255, 0, 0), 1)
            else:
                # Target too small -- stop and search
                twist.angular.z = 0.3  # Slow rotation to search
                self.get_logger().debug('Target too small, searching...')
        else:
            # No target found -- stop and search
            twist.angular.z = 0.3
            self.get_logger().debug('No target, searching...')

        self.cmd_vel_pub.publish(twist)

        # Publish debug image
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.debug_pub.publish(debug_msg)
        except Exception:
            pass

    def destroy_node(self):
        """Stop on shutdown."""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VisualTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
