import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class RectanglePublisher(Node):
    def __init__(self):
        super().__init__('rectangle_publisher')
        
        # Create publisher for turtle velocity commands
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        # Timer to publish commands at regular intervals
        timer_period = 0.1  # seconds (10 Hz)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Rectangle parameters
        self.length = 4.0  # Length of rectangle
        self.width = 2.0   # Width of rectangle
        self.linear_velocity = 2.0   # Linear speed (units/sec)
        self.angular_velocity = 1.0  # Angular speed for turns (rad/sec)
        
        # State tracking
        self.current_side = 0  # 0=length, 1=width, 2=length, 3=width
        self.sides_completed = 0
        self.rectangle_completed = False
        
        # Time tracking
        self.start_time = self.get_clock().now()
        self.side_start_time = self.get_clock().now()
        
        # Calculate durations
        self.length_duration = self.length / self.linear_velocity
        self.width_duration = self.width / self.linear_velocity
        self.turn_duration = (math.pi / 2) / self.angular_velocity  # 90 degree turn
        
        # State machine: 'moving' or 'turning'
        self.state = 'moving'
        self.turn_start_time = None
        
        self.get_logger().info('Rectangle drawing node started. The turtle will draw one complete rectangle.')
        self.get_logger().info(f'Rectangle dimensions: {self.length} x {self.width} units')
        self.get_logger().info(f'Length duration: {self.length_duration:.2f}s, Width duration: {self.width_duration:.2f}s')
        self.get_logger().info(f'Turn duration: {self.turn_duration:.2f}s')

    def timer_callback(self):
        if self.rectangle_completed:
            return
        
        current_time = self.get_clock().now()
        
        if self.state == 'moving':
            self.handle_moving_state(current_time)
        elif self.state == 'turning':
            self.handle_turning_state(current_time)

    def handle_moving_state(self, current_time):
        # Determine current side duration
        if self.current_side % 2 == 0:  # sides 0 and 2 are length
            side_duration = self.length_duration
            side_name = "length"
        else:  # sides 1 and 3 are width
            side_duration = self.width_duration
            side_name = "width"
        
        elapsed = (current_time - self.side_start_time).nanoseconds / 1e9
        
        # Check if current side is completed
        if elapsed >= side_duration:
            self.stop_turtle()
            self.sides_completed += 1
            self.get_logger().info(f'Completed {side_name} side {self.sides_completed}/4')
            
            # Check if rectangle is completed
            if self.sides_completed >= 4:
                self.rectangle_completed = True
                self.get_logger().info('Rectangle completed! Turtle has returned to starting position.')
                return
            
            # Start turning for next side
            self.state = 'turning'
            self.turn_start_time = current_time
            self.current_side = (self.current_side + 1) % 4
            return
        
        # Continue moving straight
        self.move_straight()

    def handle_turning_state(self, current_time):
        elapsed = (current_time - self.turn_start_time).nanoseconds / 1e9
        
        # Check if turn is completed
        if elapsed >= self.turn_duration:
            self.stop_turtle()
            self.get_logger().info(f'Completed 90° turn, starting side {self.sides_completed + 1}/4')
            
            # Start moving the next side
            self.state = 'moving'
            self.side_start_time = current_time
            return
        
        # Continue turning
        self.turn_left()

    def move_straight(self):
        """Move the turtle straight forward"""
        msg = Twist()
        msg.linear.x = self.linear_velocity
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)

    def turn_left(self):
        """Turn the turtle left (counterclockwise)"""
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = self.angular_velocity  # Positive for counterclockwise
        self.publisher_.publish(msg)

    def stop_turtle(self):
        """Stop the turtle by sending zero velocities"""
        stop_msg = Twist()
        # All velocities are already 0.0 by default
        self.publisher_.publish(stop_msg)

def main(args=None):
    # Initialize ROS2
    rclpy.init(args=args)
    
    # Create the node
    rectangle_publisher = RectanglePublisher()
    
    try:
        # Spin the node until the rectangle is completed
        while rclpy.ok() and not rectangle_publisher.rectangle_completed:
            rclpy.spin_once(rectangle_publisher, timeout_sec=0.1)
        
        # Keep the node alive briefly after completion
        rectangle_publisher.get_logger().info('Rectangle drawing complete. Shutting down...')
        
    except KeyboardInterrupt:
        rectangle_publisher.get_logger().info('Rectangle drawing interrupted by user')
    finally:
        # Clean shutdown
        rectangle_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
