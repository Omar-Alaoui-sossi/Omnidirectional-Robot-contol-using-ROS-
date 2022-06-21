#! /usr/bin/env python3
__author__='Omar ALAOUI SOSSI'
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from tf import transformations
import numpy as np
import math


vel=0
yaw=0
max_vel_x=4
max_vel_y=4
max_angular_vel_z=4
base_diag=0.31112698372*2
wheel_radius=0.127



def vel_callback(c_vel):
	global vel 
	vel=c_vel
	
def clbk_odom(msg):
    global yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw= euler[2]

def main():
	global x_dot
	global y_dot
	global theta_dot
	global vel,yaw

	print("####################################################")
	print("-----------INVERSE KINEMATICS INITIALIZED----------")
	print("####################################################")

	x_dot = 0
	y_dot = 0
	theta_dot = 0
	vel=Twist()
	omega1 = Float64()
	omega2 = Float64()
	omega3 = Float64()
	omega4 = Float64()

	sub=rospy.Subscriber("/cmd_vel",Twist,vel_callback)
	pub1=rospy.Publisher("/final/front_right_joint_velocity_controller/command",Float64,queue_size=10)
	pub2=rospy.Publisher("/final/front_left_joint_velocity_controller/command",Float64,queue_size=10)
	pub3=rospy.Publisher("/final/back_right_joint_velocity_controller/command",Float64,queue_size=10)
	pub4=rospy.Publisher("/final/back_left_joint_velocity_controller/command",Float64,queue_size=10)
	

	while not rospy.is_shutdown():

		rospy.init_node("inverse_kinematics",anonymous=False)
		rate = rospy.Rate(10)

		if (vel.linear.x < -max_vel_x):
			x_dot = -max_vel_x
		elif  (abs(vel.linear.x) < max_vel_x):
			x_dot = vel.linear.x
		else:
			x_dot = max_vel_x

		if (vel.linear.y < -max_vel_y):
			y_dot = -max_vel_y
		elif (abs(vel.linear.y) < max_vel_y):
			y_dot = vel.linear.y
		else:
			y_dot = max_vel_y
		if (vel.angular.z > max_angular_vel_z):
			theta_dot = max_angular_vel_z
			
		elif (vel.angular.z < -max_angular_vel_z):
			theta_dot = -max_angular_vel_z
			
		else:
			theta_dot = vel.angular.z
			
		omega1.data = (-np.sin(yaw+1.75*math.pi)*x_dot + np.cos(yaw+1.75*math.pi)*y_dot + base_diag*theta_dot)/wheel_radius
		omega2.data = (-np.sin(yaw+0.25*math.pi)*x_dot + np.cos(yaw+0.25*math.pi)*y_dot + base_diag*theta_dot)/wheel_radius
		omega3.data = (-np.sin(yaw+1.25*math.pi)*x_dot + np.cos(yaw+1.25*math.pi)*y_dot + base_diag*theta_dot)/wheel_radius
		omega4.data = (-np.sin(yaw+0.75*math.pi)*x_dot + np.cos(yaw+0.75*math.pi)*y_dot + base_diag*theta_dot)/wheel_radius
		

		pub1.publish(omega1)                                          
		pub2.publish(omega2)                                          
		pub3.publish(omega3)                                         
		pub4.publish(omega4)
		rate.sleep()


if __name__ == '__main__':
	main()



