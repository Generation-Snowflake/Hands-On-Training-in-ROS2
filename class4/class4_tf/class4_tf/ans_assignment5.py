import math

from geometry_msgs.msg import Twist, TransformStamped

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException,TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from turtlesim.srv import Spawn


class TurtleCoordinating(Node):

    def __init__(self,coordinates):
        super().__init__('ans_assignment5')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.broadcast_timer_callback)
        self.target_x = 0.0
        self.target_y = 0.0
        self.distance_x = 100.0
        self.distance_y = 100.0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # Call on_timer function every second
        self.timer = self.create_timer(2.0, self.broadcast_timer_callback)
        # Call on_timer function every second
        self.distance_timer = self.create_timer(1.0, self.on_timer)
        self.index = 0
        self._coordinates = coordinates
    def broadcast_timer_callback(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'frame1'
        t.transform.translation.x = self._coordinates[self.index][0]
        t.transform.translation.y = self._coordinates[self.index][1]
        t.transform.translation.z = 0.0

        self.tf_broadcaster.sendTransform(t)
    def on_timer(self):
        # Store frame names in variables that will be used to
        # compute transformations
        from_frame_rel = 'frame1'
        to_frame_rel = 'turtle1'
        print("Test")
        # Look up for the transformation between target_frame and turtle2 frames
        # and send velocity commands for turtle2 to reach target_frame
        try:
            tranform_frame1_turtle1 = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())
            self.distance_x = tranform_frame1_turtle1.transform.translation.x
            self.distance_y = tranform_frame1_turtle1.transform.translation.y
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return
        if self.distance_x < 0.5 and self.distance_y < 0.5:
            self.get_logger().info(f"Change position {self.index}")
            if self.index >= 3:
                self.index = 0
            else:
                self.index += 1
    def set_target(self,target_x, target_y):
        self.target_x = target_x
        self.target_y = target_y

    def get_distance(self):
        return (self.distance_x,self.distance_y)
def main():
    coordinates = [
        (1.0, 1.5),
        (3.0, 2.0),
        (8.5, 3.2),
        (5.1, 1.2)
    ]
    rclpy.init()
    node = TurtleCoordinating(coordinates)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()