#!/usr/bin/python3
"""Publish /robot_description from a URDF file with transient_local QoS."""

from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String


class RobotDescriptionPublisher(Node):
    def __init__(self) -> None:
        super().__init__('robot_description_publisher')

        self.declare_parameter('urdf_file', '')
        urdf_file = self.get_parameter('urdf_file').get_parameter_value().string_value
        if not urdf_file:
            self.get_logger().error('urdf_file parameter is required')
            rclpy.shutdown()
            return

        urdf_path = Path(urdf_file)
        if not urdf_path.exists():
            self.get_logger().error(f'URDF file not found: {urdf_file}')
            rclpy.shutdown()
            return

        self._xml = urdf_path.read_text(encoding='utf-8')

        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE

        self.pub = self.create_publisher(String, 'robot_description', qos)
        self.pub.publish(String(data=self._xml))
        self.get_logger().info(f'Published robot_description from {urdf_file}')

        # Periodically republish in case late subscribers join.
        self.create_timer(2.0, self._republish)

    def _republish(self) -> None:
        self.pub.publish(String(data=self._xml))


def main() -> None:
    rclpy.init()
    node = RobotDescriptionPublisher()
    if rclpy.ok():
        rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
