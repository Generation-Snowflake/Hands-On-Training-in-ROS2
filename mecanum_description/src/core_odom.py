import rclpy
from rclpy.node import Node

from rclpy.time import Time

from geometry_msgs.msg import Twist, Pose2D, TransformStamped
from std_msgs.msg import Float32, Int32

from nav_msgs.msg import Odometry

import math

import tf2_ros

class Robot(Node):
	def __init__(self):
		super().__init__('robot_core_node')
		self.wheel_radius = 0.06096 #m
		self.wheelbase = 0.225 #m
		
		self.max_rpm = 60.0 #rpm
		
		self.vel_sub = self.create_subscription(Twist, 'cmd_vel', self.vel_callback,10)
		
		self.leftwheel_pub = self.create_publisher(Float32, 'wheel_command_left', 10)
		self.rightwheel_pub = self.create_publisher(Float32, 'wheel_command_right', 10)
		
		self.v = 0.0
		self.w = 0.0
		
		self.timer = self.create_timer(0.1,self.timer_callback)
		
		self.cmd_l = Float32()
		self.cmd_r = Float32()
		
		self.left_tick_sub = self.create_subscription(Int32, 'left_tick', self.left_tick_cb, 10)
		self.right_tick_sub = self.create_subscription(Int32, 'right_tick', self.right_tick_cb, 10)
		
		self.left_tick = {"bf":None, "now":None}
		self.right_tick = {"bf":None, "now":None}
		
		self.left_tick_ts = None
		self.right_tick_ts = None
		
		self.tick_per_rev = 2800
		self.robot_pose = Pose2D()
		
		self.dx = Float32()
		self.dy = Float32()
		
		self.dtheta = Float32()
		self.last_time = self.get_clock().now()
		
		self.odom_pub = self.create_publisher(Odometry, 'odom/raw', 10)
		
		self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
		
		
		#velocity variables
		self.feedback_sub = self.create_subscription(Twist, 'feedback_vel', self.feedback_vel_cb, 10)
		self.center_vel = 0.0
		self.omega = 0.0
		
		self.get_logger().info("robot core is running")
		
		
		
		
		
	def vel_callback(self, vel):
		print("callback function")
		self.v = vel.linear.x
		self.w = vel.angular.z
		
		L = self.wheelbase
		R = self.wheel_radius
		
		v_r = ((2.0*self.v)+(self.w*L))/(2.0*R)
		v_l = ((2.0*self.v)-(self.w*L))/(2.0*R)
		
		self.wheel_speed_setpoint(v_r,v_l)
	def timer_callback(self):
		self.leftwheel_pub.publish(self.cmd_l)
		self.rightwheel_pub.publish(self.cmd_r)
	
	def wheel_speed_setpoint(self,vr,vl):
		rpm_r = vr*9.549297
		rpm_l = vl*9.549297  #1 rad/s = 9.549297 rpm
		
		rpm_r = max(min(rpm_r,self.max_rpm), -self.max_rpm)
		rpm_l = max(min(rpm_l,self.max_rpm), -self.max_rpm)
		
		if rpm_r == 0.0:
			self.cmd_r.data = 0.0
		if rpm_l == 0.0:
		 	self.cmd_l.data = 0.0
		self.cmd_r.data = rpm_r
		self.cmd_l.data = rpm_l	
		
		self.get_logger().info('out left rpm: %s' %rpm_l)
		self.get_logger().info('out right rpm: %s' %rpm_r)
		
	def left_tick_cb(self, tick):
		self.left_tick["now"] = tick.data
		self.left_tick_ts = self.get_clock().now()
		
		self.odometry()
	
	def right_tick_cb(self, tick):
		self.right_tick["now"] = tick.data
		self.right_tick_ts = self.get_clock().now()
		
	def get_tick(self):
		tick = {}
		tick["l"] = self.left_tick["now"]
		tick["r"] = self.right_tick["now"]
		tick["lts"] = self.left_tick_ts
		tick["rts"] = self.right_tick_ts
		
		return tick
		
	def quaternion_from_euler(self,roll,pitch,yaw):
		cy = math.cos(yaw*0.5)
		sy = math.sin(yaw*0.5)
		cp = math.cos(pitch*0.5)
		sp = math.sin(pitch*0.5)
		cr = math.cos(roll*0.5)
		sr = math.sin(roll*0.5)
		
		q = [0]*4
		q[0] = cy * cp * cr + sy * sp * sr
		q[1] = cy * cp * sr - sy * sp * cr
		q[2] = sy * cp * sr + cy * sp * cr
		q[3] = sy * cp * cr - cy * sp * sr
		
		return q
		
	def feedback_vel_cb(self,fb):
		self.center_vel = (fb.linear.x*0.10472 + fb.linear.y*0.10472)*0.5
		self.omega = (fb.linear.y*0.10472-fb.linear.x*0.10472)/self.wheelbase
		
	def odometry(self):
		ts = self.get_clock().now()
		tick = self.get_tick()
		
		dt = ts-self.last_time
		
		dt = dt.nanoseconds*1e-9
		
		if tick['r']==None or tick['l']==None:
			return
		if self.right_tick["bf"] ==None or self.left_tick["bf"] == None:
			self.right_tick["bf"]=tick["r"]
			self.left_tick["bf"] = tick["l"]
			
		prev_robot_pose = self.robot_pose
		
		R = self.wheel_radius
		L = self.wheelbase
		
		TPR = self.tick_per_rev
		
		DPT = (2*math.pi*R)/TPR
		
		DL = DPT*(tick["l"]-self.left_tick["bf"])
		DR = DPT*(tick["r"]-self.right_tick["bf"])
		
		DC = (DL+DR)/2
		
		self.dx = DC * math.cos(prev_robot_pose.theta)
		self.dy = DC * math.sin(prev_robot_pose.theta)
		
		self.dtheta = (DR-DL)/L
		
		new_robot_pose = Pose2D()
		
		new_robot_pose.x = prev_robot_pose.x + self.dx
		new_robot_pose.y = prev_robot_pose.y + self.dy
		
		theta_tmp = prev_robot_pose.theta + self.dtheta
		
		new_robot_pose.theta = math.atan2(math.sin(theta_tmp),math.cos(theta_tmp))
		
		#update x y theta
		self.robot_pose.x = new_robot_pose.x
		self.robot_pose.y = new_robot_pose.y
		self.robot_pose.theta = new_robot_pose.theta
		
		self.left_tick["bf"] = tick["l"]
		self.right_tick["bf"] = tick["r"]
		
		#velocity calculation
		vx = self.center_vel*math.cos(prev_robot_pose.theta)
		vy = 0.0
		w = self.omega
		
		odom = Odometry()
		
		pose = self.robot_pose
		
		quat = self.quaternion_from_euler(0,0,pose.theta)
		
		odom.pose.pose.position.x = pose.x
		odom.pose.pose.position.y = pose.y
		odom.pose.pose.position.z = 0.0
		
		odom.pose.pose.orientation.w = quat[0]
		odom.pose.pose.orientation.x = quat[1]
		odom.pose.pose.orientation.y = quat[2]
		odom.pose.pose.orientation.z = quat[3]
		
		#time stamp and frame id
		odom.header.stamp = self.get_clock().now().to_msg()
		odom.header.frame_id = "odom"
		odom.child_frame_id = "base_link"
		
		#velocity odom 
		odom.twist.twist.linear.x = vx
		odom.twist.twist.linear.y = vy
		odom.twist.twist.linear.z = 0.0
		
		odom.twist.twist.angular.x = 0.0
		odom.twist.twist.angular.y = 0.0
		odom.twist.twist.angular.z = w
		odom.pose.covariance[0] = 0.001
		odom.pose.covariance[7] = 0.001
		odom.pose.covariance[35] = 0.001
		
		odom.twist.covariance[0] = 0.0001
		odom.twist.covariance[7] = 0.0001
		odom.twist.covariance[35] = 0.0001
		
		self.odom_pub.publish(odom)
		
		#TF
		t = TransformStamped()
		t.header.frame_id = "odom"
		t.child_frame_id = "base_link"
		t.header.stamp = self.get_clock().now().to_msg()
		t.transform.translation.x = pose.x
		t.transform.translation.y = pose.y
		t.transform.translation.z = 0.0
		t.transform.rotation.w = quat[0]
		t.transform.rotation.x = quat[1]
		t.transform.rotation.y = quat[2]
		t.transform.rotation.z = quat[3]
		#self.tf_broadcaster.sendTransform(t)
		
		self.last_time = ts
		
		
def main():
	rclpy.init()
	
	rb = Robot()
	rclpy.spin(rb)
	
	rclpy.shutdown()
	
if __name__ == "__main__":
	main()