#! /usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from tf import transformations
import numpy as np
import math
import sys



position=Point()
theta=0
vel=0
max_vel_x=2
max_vel_y=2
max_angular_vel_z=1.5
base_diag=0.31112698372*2
wheel_radius=0.127
goal=Point()
distance_precision=0.3
distance=0.0



	
def vel_callback(vel_callback):
	global vel
	vel=vel_callback
	print(vel_callback)

def odom_callback(msg):
	global position
	global theta
	position_ = msg.pose.pose.position
	quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
	euler = transformations.euler_from_quaternion(quaternion)
	theta = euler[2]


def speed(pos_x,pos_y):
	global x_dot
	global y_dot
	global theta_dot
	global vel
	global theta,alpha
	global position,distance

	alpha=math.atan2(pos_y - position.y, pos_x - position.x) 
	distance=math.sqrt((pos_y-position.y)**2+(pos_x-position.x))
	x_dot = 0
	y_dot = 0
	theta_dot = 0
	vel=Twist()

	if (vel.linear.x < -max_vel_x):
		x_dot = -max_vel_x*np.sin(alpha)
	elif  (abs(vel.linear.x) < max_vel_x):
		x_dot = vel.linear.x*np.sin(alpha)
	else:
		x_dot = max_vel_x*np.sin(alpha)

	if (vel.linear.y < -max_vel_y):
		y_dot = -max_vel_y*np.cos(alpha)
	elif (abs(vel.linear.y) < max_vel_y):
		y_dot = vel.linear.y*np.cos(alpha)
	else:
		y_dot = max_vel_y*np.cos(alpha)

	if (vel.angular.z > max_angular_vel_z):
		theta_dot = max_angular_vel_z
			
	elif (vel.angular.z < -max_angular_vel_z):
		theta_dot = -max_angular_vel_z
			
	else:
		theta_dot = vel.angular.z


	return x_dot,y_dot,theta_dot



	
def main(goal_x,goal_y):
	global x_dot
	global y_dot
	global theta_dot
	global vel
	global theta,alpha
	global position


	alpha=math.atan2(goal_y - position.y, goal_x - position.x) 
	distance=math.sqrt((goal_y-position.y)**2+(goal_x-position.x))
	x_dot = 0
	y_dot = 0
	theta_dot = 0
	vel=Twist()
	omega1 = Float64()
	omega2 = Float64()
	omega3 = Float64()
	omega4 = Float64()

	sub_vel=rospy.Subscriber("/cmd_vel",Twist,vel_callback)
	sub_odom=rospy.Subscriber("/odom",Odometry,odom_callback)


	pub1=rospy.Publisher("/final/front_right_joint_velocity_controller/command",Float64,queue_size=10)
	pub2=rospy.Publisher("/final/front_left_joint_velocity_controller/command",Float64,queue_size=10)
	pub3=rospy.Publisher("/final/back_right_joint_velocity_controller/command",Float64,queue_size=10)
	pub4=rospy.Publisher("/final/back_left_joint_velocity_controller/command",Float64,queue_size=10)	
		

	while not rospy.is_shutdown():

		rospy.init_node("inverse_kinematics",anonymous=False)
		rate = rospy.Rate(50)
		x_dot,y_dot,theta_dot=speed(goal_x,goal_y)
		omega1.data = (-np.sin(theta+1.75*math.pi)*x_dot + np.cos(theta+1.75*math.pi)*y_dot + base_diag*theta_dot)/wheel_radius
		omega2.data = (-np.sin(theta+0.25*math.pi)*x_dot + np.cos(theta+0.25*math.pi)*y_dot + base_diag*theta_dot)/wheel_radius
		omega3.data = (-np.sin(theta+1.25*math.pi)*x_dot + np.cos(theta+1.25*math.pi)*y_dot + base_diag*theta_dot)/wheel_radius
		omega4.data = (-np.sin(theta+0.75*math.pi)*x_dot + np.cos(theta+0.75*math.pi)*y_dot + base_diag*theta_dot)/wheel_radius
			
		if distance > distance_precision:
			pub1.publish(omega1)
			pub2.publish(omega2)
			pub3.publish(omega3)
			pub4.publish(omega4)
		else :
			pub1.publish(0)
			pub2.publish(0)
			pub3.publish(0)
			pub4.publish(0)

		rate.sleep()



if __name__ == '__main__':
	main(float(sys.argv[1]),float(sys.argv[2]))



