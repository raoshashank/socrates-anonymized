#!/usr/bin/env python3
import os
import sys
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class SelfShutdownNode(Node):
    def __init__(self):
        super().__init__('self_shutdown_node')
        self.get_logger().info('Started. I will exit in ~3 seconds...')
        self.timer = self.create_timer(10.0, self.timer_callback)
        self.pub_timer = self.create_timer(0.5, self.pub_timer_callback)
        self.publisher = self.create_publisher(Int32, 'chatter', 10)
        self.count = 0
        self.status = 0
    def pub_timer_callback(self):
        self.count+=1
        msg = Int32()
        msg.data = -15 + self.count
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        
    def timer_callback(self):
        self.get_logger().info('Timer expired, shutting down rclpy...')
        self.timer.cancel()
        self.status = 1


def main():
    # Init
    rclpy.init()
    node = SelfShutdownNode()

    # Spin in small steps (no timers/threads)
    t0 = time.time()
    try:
        while rclpy.ok() and node.status == 0:
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass

    # Clean teardown
    node.get_logger().info('Destroying Node.')
    node.destroy_node()

    # try:
    #     rclpy.shutdown()
    # except Exception:
    #     pass

    # Final guarantees: give DDS a moment, then nuke if needed.
    # time.sleep(0.1)
    # sys.stdout.flush(); sys.stderr.flush()

    # # If we ever get stuck here on your machine, this will still exit the process.
    # os._exit(0)


if __name__ == '__main__':
    main()
