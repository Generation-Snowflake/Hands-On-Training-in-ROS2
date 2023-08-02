from geometry_msgs.msg import TransformStamped

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster


class FixedFrameBroadcaster(Node):

   def __init__(self):
       super().__init__('assignment3_broadcaster')
       self.tf_broadcaster = TransformBroadcaster(self)
       self.timer = self.create_timer(0.1, self.broadcast_timer_callback)
       self.target_x = self.declare_parameter('x').get_parameter_value().double_value
       self.target_y = self.declare_parameter('y').get_parameter_value().double_value

   def broadcast_timer_callback(self):
       t = TransformStamped()

       t.header.stamp = self.get_clock().now().to_msg()
       t.header.frame_id = 'world'
       t.child_frame_id = 'frame1'
       t.transform.translation.x = self.target_x
       t.transform.translation.y = self.target_y
       t.transform.translation.z = 0.0

       self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = FixedFrameBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()