#! /usr/bin/env python3
__author__="Omar ALAOUI SOSSI"

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from tf import transformations
from gazebo_msgs.srv import SetPhysicsProperties


def callback(msg):
	pass

def main():
	rospy.init_node("move_omni",anonymous=False)
	rate=rospy.Rate(1)
	
#---------------------- publisher setting ------------------------------------------------------------#
	pub1=rospy.Publisher("/final/front_right_joint_velocity_controller/command",Float64,queue_size=10)
	pub2=rospy.Publisher("/final/front_left_joint_velocity_controller/command",Float64,queue_size=10)
	pub3=rospy.Publisher("/final/back_right_joint_velocity_controller/command",Float64,queue_size=10)
	pub4=rospy.Publisher("/final/back_left_joint_velocity_controller/command",Float64,queue_size=10)

	
	while not rospy.is_shutdown():
		pub1.publish(-10.0)                                          #forawards 10  -10    10    -10 
		pub2.publish(0.0)                                           #backwards -10  10    -10    10
		pub3.publish(0.0)                                          #rotation clock wise   same value all wheels   
		pub4.publish(-20.0)                                          #rotation anti clock wise  minus oposit value value all wheels  
		rate.sleep()                                               #diagonal chose the diagonal wheels give same value the 2 left give 0 friction low no problem 


if __name__ == '__main__':
    main()
