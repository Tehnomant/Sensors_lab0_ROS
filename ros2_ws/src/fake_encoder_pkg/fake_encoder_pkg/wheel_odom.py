import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math


class WheelOdom(Node):
    def __init__(self):
        super().__init__('wheel_odom')
        
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_base', 0.30)
        
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base = self.get_parameter('wheel_base').value
        
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self._listener_callback,
            10
        )
        
        self.odom_publisher = self.create_publisher(
            Odometry,
            '/odom',
            10
        )
        
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Keep existing state variables
        self.prev_left = None
        self.prev_right = None
        self.prev_time = None
        
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        
        self.get_logger().info(
            f'Wheel odometry started with: wheel_radius={self.wheel_radius}m, '
            f'wheel_base={self.wheel_base}m'
        )

    def _listener_callback(self, msg):
        left = msg.position[0]
        right = msg.position[1]
        
        current_time = self.get_clock().now()
        
        # Initialize on first call
        if self.prev_left is None:
            self.prev_left = left
            self.prev_right = right
            self.prev_time = current_time
            return
        
        # Calculate deltas
        dtheta_l = left - self.prev_left
        dtheta_r = right - self.prev_right
        
        ds_l = self.wheel_radius * dtheta_l
        ds_r = self.wheel_radius * dtheta_r
        
        ds = (ds_r + ds_l) / 2.0
        dpsi = (ds_r - ds_l) / self.wheel_base
        
        # Update pose using midpoint integration
        self.x += ds * math.cos(self.yaw + dpsi / 2.0)
        self.y += ds * math.sin(self.yaw + dpsi / 2.0)
        self.yaw += dpsi
        
        # Normalize yaw to [-π, π]
        self.yaw = self.normalize_angle(self.yaw)
        
        # Calculate time difference
        dt = (current_time - self.prev_time).nanoseconds * 1e-9
        
        # Calculate velocities
        if dt > 0:
            v = ds / dt
            omega = dpsi / dt
        else:
            v = 0.0
            omega = 0.0
        
        self.publish_odometry(current_time, v, omega)
        
        self.publish_transform(current_time)
        
        # Update previous values
        self.prev_left = left
        self.prev_right = right
        self.prev_time = current_time
        
        self.get_logger().info(
            f"Odom: Position=({self.x:.3f}, {self.y:.3f}), "
            f"Orientation={self.yaw:.3f}, "
            f"Linear Velocity={v:.3f}, Angular Velocity={omega:.3f}"
        )
    
    def normalize_angle(self, angle):
        """Normalize angle to range [-π, π]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def publish_odometry(self, current_time, v, omega):
        """Publish odometry message as required in Task 4.1 & 4.4"""
        odom_msg = Odometry()
        
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        
        q = self.yaw_to_quaternion(self.yaw)
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]
        
        # 6x6 matrix (position: x, y, z, orientation: roll, pitch, yaw)
        odom_msg.pose.covariance = [0.0] * 36
        odom_msg.pose.covariance[0] = 0.01   # x variance
        odom_msg.pose.covariance[7] = 0.01   # y variance  
        odom_msg.pose.covariance[35] = 0.02  # yaw variance
        
        odom_msg.twist.twist.linear.x = v
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = omega
        
        odom_msg.twist.covariance = [0.0] * 36
        odom_msg.twist.covariance[0] = 0.01   # linear x variance
        odom_msg.twist.covariance[35] = 0.02  # angular z variance
        
        self.odom_publisher.publish(odom_msg)
    
    def publish_transform(self, current_time):
        """Publish TF transform odom → base_link as required in Task 4.1 & 4.5"""
        transform = TransformStamped()
        
        # Set header
        transform.header.stamp = current_time.to_msg()
        transform.header.frame_id = 'odom'      # Parent frame
        transform.child_frame_id = 'base_link'  # Child frame
        
        # Set translation
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        
        # Set rotation (convert yaw to quaternion)
        q = self.yaw_to_quaternion(self.yaw)
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]
        
        self.tf_broadcaster.sendTransform(transform)
    
    def yaw_to_quaternion(self, yaw):
        """Convert yaw angle to quaternion (x=0, y=0, z=sin(yaw/2), w=cos(yaw/2))"""
        return [
            0.0,  # x
            0.0,  # y
            math.sin(yaw / 2.0),  # z
            math.cos(yaw / 2.0)   # w
        ]


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdom()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Wheel odometry node shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()