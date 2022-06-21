#! /usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from tf import transformations
from gazebo_msgs.srv import SetPhysicsProperties
from std_srvs.srv import *
import math
import numpy as np

active=False
yaw_precision=math.pi / 90
desired_position_ = Point()
desired_position_.x = rospy.get_param('des_pos_x')
desired_position_.y = rospy.get_param('des_pos_y')
desired_position_.z = rospy.get_param('des_pos_yaw')
dist_precision_ = 0.3

def go_to_point_switch(req):
    global active
    active_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res


def clbk_odom(msg):
    global position_
    global yaw_

    position_ = msg.pose.pose.position
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]


def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

def orientation(des_pos):
	desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
	err_yaw = normalize_angle(desired_yaw - yaw_)
	vel=Twist()
	if err_yaw >= yaw_precision:
		vel.angular.z=-0.2
	else: 
		vel.angular.z=0

def go(des_pos):
	global vel ,position_
	distance=math.sqrt(pow(des_pos.y - position_.y, 2) + pow(des_pos.x - position_.x, 2))
	if distance>desired_position:
		vel=Twist()
		vel.linear.x =1.0
	else :
		vel.linear.x=0








def done():
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub.publish(twist_msg)


def main():
    global pub, active

    rospy.init_node('go_to_point')

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)

    srv = rospy.Service('go_to_point_switch', SetBool, go_to_point_switch)

    rate = rospy.Rate(20)


    while not rospy.is_shutdown():
        if not active:
        	go(desired_position_) and orientation(desired_position_)

        rate.sleep()

if __name__ == '__main__':
    main()