import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.time = 0

    def create_twist(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        return msg

    def get_twist_msg(self):
        # Drawing a hexagon
        move_duration = 4        # ticks per side
        turn_duration = 3         # ticks per 60° turn

        sides = 6                 # hexagon sides

        # Total cycle duration (6 sides)
        cycle_duration = (move_duration + turn_duration) * sides

        # Calculate how long we've been drawing current side
        time_in_phase = self.time % (move_duration + turn_duration)

        if time_in_phase < move_duration:
            # Move forward
            linear_x = 1.0
            angular_z = 0.0
        else:
            # Turn at corner
            linear_x = 0.0
            # Calculate angular velocity needed for 60 degree turn
            angular_z = (math.pi / 3) / (turn_duration * 0.5)

        # Stop after completing hexagon
        if self.time >= cycle_duration:
            linear_x = 0.0
            angular_z = 0.0

        return self.create_twist(linear_x, angular_z)
    
    def timer_callback(self):
        msg = self.get_twist_msg()       
        self.publisher.publish(msg)
        self.time += 1
        print("time: {}".format(self.time))

def main(args=None):
    rclpy.init(args=args)

    turtle_controller = TurtleController()

    rclpy.spin(turtle_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    turtle_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
