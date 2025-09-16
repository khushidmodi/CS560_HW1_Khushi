import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class CirclePublisher(Node):
    def __init__(self):
        super().__init__('circle_publisher')
        
        # Create publisher for turtle velocity commands
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        # Timer to publish commands at regular intervals
        timer_period = 0.1  # seconds (10 Hz)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Circle parameters
        self.linear_velocity = 2.0   # Linear speed (units/sec)
        self.angular_velocity = 1.0  # Angular speed (rad/sec)
        
        # Track time for one complete circle
        self.start_time = self.get_clock().now()
        self.circle_duration = 2 * math.pi / self.angular_velocity  # Time for one complete circle
        self.circle_completed = False
        
        self.get_logger().info('Circle drawing node started. The turtle will draw one complete circle.')
        self.get_logger().info(f'Circle radius: {self.linear_velocity/self.angular_velocity:.2f} units')
        self.get_logger().info(f'Time for one circle: {self.circle_duration:.2f} seconds')

    def timer_callback(self):
        # Check if circle is completed
        if self.circle_completed:
            return
        
        current_time = self.get_clock().now()
        elapsed = (current_time - self.start_time).nanoseconds / 1e9
        
        # If we've completed one full circle, stop the turtle
        if elapsed >= self.circle_duration:
            self.stop_turtle()
            self.circle_completed = True
            self.get_logger().info('Circle completed! Turtle has returned to starting position.')
            return
        
        # Create Twist message for circular motion
        msg = Twist()
        
        # Set linear velocity (forward motion)
        msg.linear.x = self.linear_velocity
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        
        # Set angular velocity (turning motion)
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = self.angular_velocity
        
        # Publish the message
        self.publisher_.publish(msg)
    
    def stop_turtle(self):
        """Stop the turtle by sending zero velocities"""
        stop_msg = Twist()
        # All velocities are already 0.0 by default
        self.publisher_.publish(stop_msg)
        self.get_logger().info('Turtle stopped.')

def main(args=None):
    # Initialize ROS2
    rclpy.init(args=args)
    
    # Create the node
    circle_publisher = CirclePublisher()
    
    try:
        # Spin the node until the circle is completed
        while rclpy.ok() and not circle_publisher.circle_completed:
            rclpy.spin_once(circle_publisher, timeout_sec=0.1)
        
        # Keep the node alive briefly after completion
        circle_publisher.get_logger().info('Circle drawing complete. Shutting down...')
        
    except KeyboardInterrupt:
        circle_publisher.get_logger().info('Circle drawing interrupted by user')
    finally:
        # Clean shutdown
        circle_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
