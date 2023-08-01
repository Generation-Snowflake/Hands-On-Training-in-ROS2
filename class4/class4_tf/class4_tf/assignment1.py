import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import math

class TurtleLine(Node):

    def __init__(self, speed):
        super().__init__('turtle_line')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.speed = speed

    def timer_callback(self):
        vel_msg = Twist()
        vel_msg.linear.x = self.speed

        self.publisher_.publish(vel_msg)
        self.get_logger().info(f"Moving with speed = {self.speed}")

def main(args=None):
    rclpy.init(args=args)

    speed = float(sys.argv[1])

    turtlesim_pub = TurtleLine(speed)

    rclpy.spin(turtlesim_pub)

    # Destroy the node explicitly
    turtlesim_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main(sys.argv)
