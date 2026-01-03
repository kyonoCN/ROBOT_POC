#!/usr/bin/python3
"""
Publish robot_description on a latched topic for RViz2 RobotModel display.

- Reads URDF from a file path passed via parameter `urdf_path`.
- Publishes the content as `std_msgs/String` on `/robot_description`.
- Uses transient-local QoS so that late-joining RViz instances still receive it.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import String


class RobotDescriptionPublisher(Node):
    def __init__(self) -> None:
        super().__init__('robot_description_publisher')

        # Declare and get URDF path parameter
        self.declare_parameter('urdf_path', '')
        urdf_path = self.get_parameter('urdf_path').get_parameter_value().string_value

        if not urdf_path:
            self.get_logger().error('Parameter `urdf_path` is empty, cannot load URDF.')
            raise RuntimeError('urdf_path parameter is required')

        try:
            with open(urdf_path, 'r') as f:
                urdf_content = f.read()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Failed to read URDF file: {urdf_path}: {exc}')
            raise

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self.pub = self.create_publisher(String, 'robot_description', qos)

        msg = String()
        msg.data = urdf_content
        self.pub.publish(msg)
        self.get_logger().info(f'Published robot_description from: {urdf_path}')

        # Optionally re-publish periodically in case someone clears the topic
        self.timer = self.create_timer(5.0, lambda: self.pub.publish(msg))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RobotDescriptionPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
