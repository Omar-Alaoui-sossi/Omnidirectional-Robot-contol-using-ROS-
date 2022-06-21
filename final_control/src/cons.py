#! /usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *
import math
import numpy as np

x=0
y=0 
theta =0 
vel=0
yaw_precision = 0.01
dist_precision_=0.3
def odom_callback(msg):
	global x,y,theta 

	x=msg.pose.pose.position.x
	y=msg.pose.pose.position.y 

	quat=(msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
	(roll,pitch,theta)= transformations.euler_from_quaternion(quat)
	

def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

def done():
    global vel
    vel = Twist()
    vel.linear.x = 0
    vel.angular.z = 0
    pub.publish(vel)

def main():
	rospy.init_node("move")
	rate=rospy.Rate(1)
	sub_odom = rospy.Subscriber('/odom', Odometry,odom_callback)
	pub=rospy.Publisher('/cmd_vel', Twist, queue_size=1)
	vel=Twist()
	goal=Point()
	goal.x=-5
	goal.y=-5
	while not rospy.is_shutdown():
		err_x=goal.x-x
		err_y=goal.y-y
		distance=math.sqrt((err_y)**2 + err_x**2)

		angle=math.atan2(err_y,err_x)
		ang=normalize_angle(angle-theta)

		if angle>yaw_precision:
			vel.linear.x=0.0
			vel.angular.z=-0.5
			pub.publish(vel)
			

		else :
			vel.linear.x=0.8
			vel.angular.z=0.0
			pub.publish(vel)
			if distance < dist_precision_:
				done() 
			
		rate.sleep()



	

if __name__=='__main__':
	main()



