#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


THRUST = 150.0
RATE_HZ = 10.0


class OtterForward(Node):
    def __init__(self):
        super().__init__('otter_forward')
        self._left = self.create_publisher(Float64, '/otter/thrusters/left/thrust', 10)
        self._right = self.create_publisher(Float64, '/otter/thrusters/right/thrust', 10)
        self.create_timer(1.0 / RATE_HZ, self._publish)
        self.get_logger().info(f'otter_forward: publishing thrust={THRUST}N at {RATE_HZ}Hz')

    def _publish(self):
        msg = Float64()
        msg.data = THRUST
        self._left.publish(msg)
        self._right.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OtterForward()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
