import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class OdomSubscriber(Node):
    def __init__(self):
        super().__init__('odom_calc')

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self._listener_callback,
            10
        )

        self.prev_left = None
        self.prev_right = None
        self.prev_time = None

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

    def _listener_callback(self, msg):
        left = msg.position[0]
        right = msg.position[1]

        if self.prev_left is None:
            self.prev_left = left
            self.prev_right = right
            self.prev_time = self.get_clock().now()
            return

        dtheta_l = left - self.prev_left
        dtheta_r = right - self.prev_right

        r = 0.05
        b = 0.30

        ds_l = r * dtheta_l
        ds_r = r * dtheta_r

        ds = (ds_r + ds_l) / 2.0
        dpsi = (ds_r - ds_l) / b

        self.x += ds * math.cos(self.yaw + dpsi / 2.0)
        self.y += ds * math.sin(self.yaw + dpsi / 2.0)
        self.yaw += dpsi

        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds * 1e-9

        if dt == 0:
            v = 0.0
            omega = 0.0
        else:
            v = ds / dt
            omega = dpsi / dt

        self.prev_left = left
        self.prev_right = right
        self.prev_time = now

        self.get_logger().info(
            f"Odom: Position=({self.x:.3f}, {self.y:.3f}), Orientation={self.yaw:.3f}, Linear Velocity={v:.3f}, Angular Velocity={omega:.3f}"
        )


def main(args=None):
    try:
        rclpy.init(args=args)
        node = OdomSubscriber()
        rclpy.spin(node)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()