import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import JointState
import math


class EncoderDriver(Node):
    def __init__(self):
        super().__init__('encoder_driver')
        
        # Declare parameter for ticks per revolution
        self.declare_parameter('ticks_per_rev', 2048)
        
        # Get the parameter value
        self.ticks_per_rev = self.get_parameter('ticks_per_rev').value
        
        # Subscriber for wheel ticks
        self.subscription = self.create_subscription(
            Int32MultiArray,
            '/wheel_ticks',
            self.ticks_callback,
            10
        )
        
        # Publisher for joint states
        self.publisher_ = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )
        
        # Store current wheel angles in radians
        self.left_wheel_angle = 0.0
        self.right_wheel_angle = 0.0
        
        # Store previous ticks for velocity calculation
        self.prev_left_ticks = 0
        self.prev_right_ticks = 0
        self.prev_time = self.get_clock().now()
        
        # Calculate radians per tick
        self.radians_per_tick = 2 * math.pi / self.ticks_per_rev
        
    
    def ticks_callback(self, msg):
        """
        Callback function for processing incoming tick messages.
        Converts ticks to wheel angles and publishes joint states.
        """
        
        current_time = self.get_clock().now()
        
        # Extract left and right ticks
        left_ticks = msg.data[0]
        right_ticks = msg.data[1]
        
        # Convert ticks to angle
        left_angle = left_ticks * self.radians_per_tick
        right_angle = right_ticks * self.radians_per_tick
        
        # Calculate time difference for velocity
        dt = (current_time - self.prev_time).nanoseconds * 1e-9  # Convert to seconds
        
        # Calculate angular velocities (rad/s)
        if dt > 0:
            left_velocity = (left_angle - self.left_wheel_angle) / dt
            right_velocity = (right_angle - self.right_wheel_angle) / dt
        else:
            left_velocity = 0.0
            right_velocity = 0.0
        
        # Create JointState message
        joint_state_msg = JointState()
        
        # Set header timestamp
        joint_state_msg.header.stamp = current_time.to_msg()
        joint_state_msg.header.frame_id = 'base_link'
        
        # Set joint names
        joint_state_msg.name = ['left_wheel_joint', 'right_wheel_joint']
        
        # Set positions
        joint_state_msg.position = [float(left_angle), float(right_angle)]
        
        # Set velocities
        joint_state_msg.velocity = [float(left_velocity), float(right_velocity)]
        
        # Publish the joint states
        self.publisher_.publish(joint_state_msg)
        
        # Store current values for next calculation
        self.left_wheel_angle = left_angle
        self.right_wheel_angle = right_angle
        self.prev_left_ticks = left_ticks
        self.prev_right_ticks = right_ticks
        self.prev_time = current_time
        
        self.get_logger().info(
            f'Angles: L={left_angle:.3f}rad, R={right_angle:.3f}rad | '
            f'Velocity: L={left_velocity:.3f}rad/s, R={right_velocity:.3f}rad/s'
        )


def main(args=None):
    rclpy.init(args=args)
    
    encoder_driver = EncoderDriver()
    
    rclpy.spin(encoder_driver)

    encoder_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()