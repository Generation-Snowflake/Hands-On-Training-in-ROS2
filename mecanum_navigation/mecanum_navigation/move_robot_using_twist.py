import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
#from tf.transformations import euler_from_quaternion
from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2

class Moverobot(Node):

    def __init__(self):
        super().__init__('Subdom')
        self.x = 0.0
        self.y = 0.0 
        self.theta = 0.0
        self.pub = self.create_publisher(Twist, "/cmd_vel",10)
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.newOdom,
            10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.goal = Point()
    def pubCmdvel(self):
        speed = Twist()
        inc_x = self.goal.x - self.x
        inc_y = self.goal.y - self.y
        angle_to_goal = atan2(inc_y, inc_x)

        if abs(angle_to_goal - self.theta) > 0.1:
            speed.linear.x = 0.0
            speed.angular.z = 0.3
        elif abs(inc_x - self.x) > 0.1 and abs(inc_y - self.y) > 0.1 :
            speed.linear.x = 0.5
            speed.angular.z = 0.0
        else:
            speed.linear.x = 0.0
            speed.angular.z = 0.0
            self.get_logger().info("SUCCEEDED")
        self.pub.publish(speed)
        #self.get_logger().info('Publishing: "%s"' % msg.data)
    def timer_callback(self):
        self.pubCmdvel()
    def newOdom(self,msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        rot_q = msg.pose.pose.orientation
        (roll, pitch, self.theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
        #self.get_logger().info('Subodom')
    def setXYGoal(self,x,y):
        self.goal.x = x
        self.goal.y = y

def main():
    rclpy.init()
    move_robot = Moverobot()
    move_robot.setXYGoal(3.0,3.0)
    try:
        rclpy.spin(move_robot)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
if __name__ == '__main__':
    main()