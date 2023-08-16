import math

from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from turtlesim.srv import Spawn


class TurtleCoordinating(Node):

    def __init__(self):
        super().__init__('week5_assignment')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create turtle2 velocity publisher
        self.publisher = self.create_publisher(Twist, 'turtle1/cmd_vel', 1)

        # Call on_timer function every second
        self.timer = self.create_timer(1.0, self.move_turtlesim)

        # Specific Co-ordinate
        self.coordinates = [
            (1.0, 1.5),
            (3.0, 2.0),
            (8.5, 3.2),
            (9.1, 1.2)
        ]

    def move_turtlesim(self):
        target_frame = "turtle1"
        reference_frame = "world"

        dummy = Twist()
        
        # Look up for the transformation between target_frame and turtle2 frames
        # and send velocity commands for turtle2 to reach target_frame
        for x, y in self.coordinates:
        
            try:
                t = self.tf_buffer.lookup_transform(
                    target_frame,
                    reference_frame,
                    rclpy.time.Time(),
                    rclpy.duration.Duration(seconds=1.0))
                dummy.linear.x = t.transform.translation.x
                dummy.linear.y = t.transform.translation.y
            except TransformException as ex:
                self.get_logger().info(
                    f'Could not transform {target_frame} to {reference_frame}: {ex}')
                return
            self.publisher.publish(dummy)

def main():
    rclpy.init()
    node = TurtleCoordinating()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()