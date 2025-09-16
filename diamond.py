import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class DiamondPublisher(Node):
    def __init__(self):
        super().__init__('diamond_publisher')
        
        # Create publisher for turtle velocity commands
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        # Timer to publish commands at regular intervals
        timer_period = 0.1  # seconds (10 Hz)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Diamond parameters
        self.side_length = 3.0  # Length of each diamond side
        self.linear_velocity = 2.0   # Linear speed (units/sec)
        self.angular_velocity = 1.0  # Angular speed for turns (rad/sec)
        
        # Diamond geometry: 4 equal sides with alternating 60° and 120° turns
        # This creates a diamond (rhombus) shape
        self.turn_angles = [math.pi * 2/3, math.pi * 1/3, math.pi * 2/3, math.pi * 1/3]  # 120°, 60°, 120°, 60° in radians
        
        # State tracking
        self.current_side = 0  # 0, 1, 2, 3 for four sides
        self.sides_completed = 0
        self.diamond_completed = False
        
        # Time tracking
        self.start_time = self.get_clock().now()
        self.side_start_time = self.get_clock().now()
        
        # Calculate durations
        self.side_duration = self.side_length / self.linear_velocity
        self.turn_durations = [angle / self.angular_velocity for angle in self.turn_angles]
        
        # State machine: 'moving' or 'turning'
        self.state = 'moving'
        self.turn_start_time = None
        
        self.get_logger().info('Diamond drawing node started. The turtle will draw one complete diamond.')
        self.get_logger().info(f'Diamond side length: {self.side_length} units')
        self.get_logger().info(f'Side duration: {self.side_duration:.2f}s')
        self.get_logger().info(f'Turn angles: 120°, 60°, 120°, 60°')

    def timer_callback(self):
        if self.diamond_completed:
            return
        
        current_time = self.get_clock().now()
        
        if self.state == 'moving':
            self.handle_moving_state(current_time)
        elif self.state == 'turning':
            self.handle_turning_state(current_time)

    def handle_moving_state(self, current_time):
        elapsed = (current_time - self.side_start_time).nanoseconds / 1e9
        
        # Check if current side is completed
        if elapsed >= self.side_duration:
            self.stop_turtle()
            self.sides_completed += 1
            self.get_logger().info(f'Completed side {self.sides_completed}/4')
            
            # Check if diamond is completed
            if self.sides_completed >= 4:
                self.diamond_completed = True
                self.get_logger().info('Diamond completed! Turtle has returned to starting position.')
                return
            
            # Start turning for next side
            self.state = 'turning'
            self.turn_start_time = current_time
            return
        
        # Continue moving straight
        self.move_straight()

    def handle_turning_state(self, current_time):
        elapsed = (current_time - self.turn_start_time).nanoseconds / 1e9
        current_turn_duration = self.turn_durations[self.current_side]
        turn_angle_degrees = math.degrees(self.turn_angles[self.current_side])
        
        # Check if turn is completed
        if elapsed >= current_turn_duration:
            self.stop_turtle()
            self.get_logger().info(f'Completed {turn_angle_degrees:.0f}° turn, starting side {self.sides_completed + 1}/4')
            
            # Move to next side
            self.current_side = (self.current_side + 1) % 4
            self.state = 'moving'
            self.side_start_time = current_time
            return
        
        # Continue turning left
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
    diamond_publisher = DiamondPublisher()
    
    try:
        # Spin the node until the diamond is completed
        while rclpy.ok() and not diamond_publisher.diamond_completed:
            rclpy.spin_once(diamond_publisher, timeout_sec=0.1)
        
        # Keep the node alive briefly after completion
        diamond_publisher.get_logger().info('Diamond drawing complete. Shutting down...')
        
    except KeyboardInterrupt:
        diamond_publisher.get_logger().info('Diamond drawing interrupted by user')
    finally:
        # Clean shutdown
        diamond_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
