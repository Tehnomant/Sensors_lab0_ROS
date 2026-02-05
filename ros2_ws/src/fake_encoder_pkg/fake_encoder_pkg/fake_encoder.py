import rclpy
from rclpy.node import Node
import random
from std_msgs.msg import Int32MultiArray


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('fake_encoder')

        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('ticks_per_rev', 2048)
        self.declare_parameter('left_rps', 1.0)
        self.declare_parameter('right_rps', 1.0)
        
        publish_rate_hz = self.get_parameter('publish_rate_hz').value
        self.ticks_per_rev = self.get_parameter('ticks_per_rev').value
        self.left_rps = self.get_parameter('left_rps').value
        self.right_rps = self.get_parameter('right_rps').value

        self.publisher_ = self.create_publisher(Int32MultiArray, '/wheel_ticks', 10)
        
        timer_period = 1.0 / publish_rate_hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.left_ticks = 0
        self.right_ticks = 0

        self.last_time = self.get_clock().now()
        
        self.left_tps = self.left_rps * self.ticks_per_rev
        self.right_tps = self.right_rps * self.ticks_per_rev

    def timer_callback(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9

        # Calculate tick increments based on wheel speeds
        left_increment = int(self.left_tps * dt)
        right_increment = int(self.right_tps * dt)

        
        # Update tick counters
        self.left_ticks += left_increment
        self.right_ticks += right_increment

        msg = Int32MultiArray()
        msg.data = [self.left_ticks, self.right_ticks]
        self.publisher_.publish(msg)

        self.last_time = current_time
        
        self.get_logger().info(f'Publishing: left={self.left_ticks}, right={self.right_ticks}')


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()