import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Point

from nav_msgs.msg import Odometry

import math

class odomNav(Node):
	def __init__(self):
		super().__init__('odom_nav_node')
		
		self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback,10)
		self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
		
		self.timer = self.create_timer(0.1, self.timer_callback)
		
		self.get_logger().info("the simple navigation node is working")
		
		self.x = 0.0
		self.y = 0.0
		self.theta = 0.0
		
		self.vel_cmd = Twist()
		
		self.goal = Point()
		
		self.goal.x = 1.0
		self.goal.y = 1.25
		
	def euler_from_quaternion(self,x,y,z,w):
		t0 = +2.0 * (w * x + y * z)
		t1 = +1.0 - 2.0 * (x * x + y * y)
		roll = math.atan2(t0, t1)
		
		t2 = +2.0 * (w * y - z * x)
		t2 = +1.0 if t2 > +1.0 else t2
		t2 = -1.0 if t2 < -1.0 else t2
		pitch = math.asin(t2)

		t3 = +2.0 * (w * z + x * y)
		t4 = +1.0 - 2.0 * (y * y + z * z)
		yaw = math.atan2(t3, t4)
		
		return roll, pitch , yaw
	
	def timer_callback(self):
		self.vel_pub.publish(self.vel_cmd)
		self.get_logger().info("X: %s" %self.x)
		self.get_logger().info("Y: %s" %self.y)

	
	def odom_callback(self,odom):
		self.navigation()
		self.x = odom.pose.pose.position.x
		self.y = odom.pose.pose.position.y
		
		qx = odom.pose.pose.orientation.x
		qy = odom.pose.pose.orientation.y
		qz = odom.pose.pose.orientation.z
		qw = odom.pose.pose.orientation.w
		
		(r,p,y) = self.euler_from_quaternion(qx,qy,qz,qw)
		self.theta = y
		
	def navigation(self):
		dx = self.goal.x - self.x
		dy = self.goal.y - self.y
		
		dxy = math.sqrt(dx**2+dy**2)
		
		dtheta = math.atan2(dy,dx)
		
		if abs(dtheta - self.theta) > 0.1:
			self.vel_cmd.linear.x = 0.0
			self.vel_cmd.angular.z = 0.5
		else:
			self.vel_cmd.linear.x = 0.3 * dxy
			self.vel_cmd.angular.z = 0.0
			
	def stop(self):
		self.vel_cmd.linear.x = 0.0
		self.vel_cmd.angular.z = 0.0
		self.vel_pub.publish(self.vel_cmd)
	
def main():
	rclpy.init()
	
	on = odomNav()
	
	try:
		rclpy.spin(on)
	except KeyboardInterrupt:
		on.stop()
		print('stopped')
	
	
	
	rclpy.shutdown()
	
if __name__=="__main__":
	main()
	
	