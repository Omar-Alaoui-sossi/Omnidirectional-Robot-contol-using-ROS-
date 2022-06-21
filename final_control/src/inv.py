#! /usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from tf import transformations
import numpy as np
import math


base_diag=0.31112698372
wheel_radius=0.127
goal_x=5
goal_y=5 
goal=(goal_x,goal_y)


class robot():
	def odom_callback(msg):
		global yaw
		global position,distance

		position= msg.pose.pose.position

		quaternion=(msg.pose.pose.orientation.x , 
			msg.pose.pose.orientation.y , 
			msg.pose.pose.orientation.z ,
			msg.pose.pose.orientation.w)
		euler = transformations.euler_from_quaternion(quaternion)
		yaw = euler[2]
		desired_yaw = math.atan2(goal_y - position.y, goal_x - position.x)
		distance= math.sqrt(pow(goal_y - position.y, 2) + pow(goal_x - position.x, 2))
		yaw_error=desired_yaw-yaw	
		

	def main():
		
		rate=rospy.Rate(10)
		pub1=rospy.Publisher("/final/front_right_joint_velocity_controller/command",Float64,queue_size=10)
		pub2=rospy.Publisher("/final/front_left_joint_velocity_controller/command",Float64,queue_size=10)
		pub3=rospy.Publisher("/final/back_right_joint_velocity_controller/command",Float64,queue_size=10)
		pub4=rospy.Publisher("/final/back_left_joint_velocity_controller/command",Float64,queue_size=10)
		sub=rospy.Subscriber("/odom",Odometry,odom_callback)

		while not rospy.is_shutdown():
			if distance>0.1 & yaw_error>0.1:
				foraward(1)
			else:
				rotation_clock_wise(1)
			rate.sleep()

	def foraward(x):
		pub1.publish(x)                                          
		pub2.publish(-x)                                           
		pub3.publish(x)                                            
		pub4.publish(-x)


	def backward(x):
		pub1.publish(-x)                                          
		pub2.publish(x)                                           
		pub3.publish(-x)                                            
		pub4.publish(x)

	def rotation_clock_wise(x):
		pub1.publish(x)                                           
		pub2.publish(x)                                           
		pub3.publish(x)                                            
		pub4.publish(x)


	def rotation_anti_clock_wise(x):
		pub1.publish(-x)                                          
		pub2.publish(-x)                                          
		pub3.publish(-x)                                            
		pub4.publish(-x)

	def diagonal_right(x):
		pub1.publish(x)                                          
		pub2.publish(0)                                           
		pub3.publish(0)                                            
		pub4.publish(x)


	def diagonal_left(x):
		pub1.publish(0)                                          
		pub2.publish(x)                                           
		pub3.publish(x)                                          
		pub4.publish(0)



if __name__ == '__main__':
	rospy.init_node(" move",anonymous=False)
	while not rospy.is_shutdown():
		rospy.Rate.sleep(1)
		robot()
	



