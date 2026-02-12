"""
Robot Description Node for ROS2 Jazzy.

Publishes static TF transforms describing the Szuflada V2 rover frame:
    - base_link (center of the robot)
    - ultrasonic_link (HC-SR04 sensor, front of robot)
    - camera_link (Intel RealSense D435i)

Adjust the transform values to match your physical rover layout.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster


class RobotDescriptionNode(Node):
    """Publishes static TF frames for the rover."""

    def __init__(self):
        super().__init__('robot_description')

        # Parameters for sensor positions (meters, relative to base_link)
        # Adjust these to match your physical rover
        self.declare_parameter('ultrasonic_x', 0.10)
        self.declare_parameter('ultrasonic_y', 0.0)
        self.declare_parameter('ultrasonic_z', 0.05)

        self.declare_parameter('camera_x', 0.08)
        self.declare_parameter('camera_y', 0.0)
        self.declare_parameter('camera_z', 0.12)

        ultrasonic_x = self.get_parameter('ultrasonic_x').get_parameter_value().double_value
        ultrasonic_y = self.get_parameter('ultrasonic_y').get_parameter_value().double_value
        ultrasonic_z = self.get_parameter('ultrasonic_z').get_parameter_value().double_value

        camera_x = self.get_parameter('camera_x').get_parameter_value().double_value
        camera_y = self.get_parameter('camera_y').get_parameter_value().double_value
        camera_z = self.get_parameter('camera_z').get_parameter_value().double_value

        self.static_broadcaster = StaticTransformBroadcaster(self)

        transforms = []

        # base_footprint -> base_link (ground to center of robot body)
        t_base = TransformStamped()
        t_base.header.stamp = self.get_clock().now().to_msg()
        t_base.header.frame_id = 'base_footprint'
        t_base.child_frame_id = 'base_link'
        t_base.transform.translation.x = 0.0
        t_base.transform.translation.y = 0.0
        t_base.transform.translation.z = 0.03  # Clearance above ground
        t_base.transform.rotation.w = 1.0
        transforms.append(t_base)

        # base_link -> ultrasonic_link (front of rover)
        t_ultra = TransformStamped()
        t_ultra.header.stamp = self.get_clock().now().to_msg()
        t_ultra.header.frame_id = 'base_link'
        t_ultra.child_frame_id = 'ultrasonic_link'
        t_ultra.transform.translation.x = ultrasonic_x
        t_ultra.transform.translation.y = ultrasonic_y
        t_ultra.transform.translation.z = ultrasonic_z
        t_ultra.transform.rotation.w = 1.0
        transforms.append(t_ultra)

        # base_link -> camera_link (RealSense D435i position)
        t_cam = TransformStamped()
        t_cam.header.stamp = self.get_clock().now().to_msg()
        t_cam.header.frame_id = 'base_link'
        t_cam.child_frame_id = 'camera_link'
        t_cam.transform.translation.x = camera_x
        t_cam.transform.translation.y = camera_y
        t_cam.transform.translation.z = camera_z
        t_cam.transform.rotation.w = 1.0
        transforms.append(t_cam)

        # base_link -> laser_frame (virtual laser from depth camera)
        # Same position as camera but used by depthimage_to_laserscan
        t_laser = TransformStamped()
        t_laser.header.stamp = self.get_clock().now().to_msg()
        t_laser.header.frame_id = 'base_link'
        t_laser.child_frame_id = 'laser_frame'
        t_laser.transform.translation.x = camera_x
        t_laser.transform.translation.y = camera_y
        t_laser.transform.translation.z = camera_z
        t_laser.transform.rotation.w = 1.0
        transforms.append(t_laser)

        self.static_broadcaster.sendTransform(transforms)
        self.get_logger().info(
            f'Published static TF: base_footprint -> base_link, '
            f'base_link -> ultrasonic_link ({ultrasonic_x},{ultrasonic_y},{ultrasonic_z}), '
            f'base_link -> camera_link ({camera_x},{camera_y},{camera_z}), '
            f'base_link -> laser_frame'
        )


def main(args=None):
    rclpy.init(args=args)
    node = RobotDescriptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
