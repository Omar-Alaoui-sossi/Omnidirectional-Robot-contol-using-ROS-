#! /usr/bin/env python3
__author__="Omar ALAOUI SOSSI"

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *
import sys
import math
import numpy as np 

position = Point()
yaw = 0
state_ = 0
yaw_precision= math.pi / 90 # +/- 2 degree allowed
dist_precision= 0.32568956884 #for mmore precision increase the rospy Rate
pub = None
alpha=0
vel_max=2


print('###########')
print("Omni drive activated ")
print('###########')

#inorder to have an angle that is always between pi and -pi
def normalize_angle(angle): 
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

def clbk_odom(msg):
    global position
    global yaw
    position = msg.pose.pose.position
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw = euler[2] #robor orientation
   
def go(des_pos_x,des_pos_y):# the parameteres  of this function are the point of the goal ,it will be given by the user using the sys library

    global yaw, pub, yaw_precision, state_,alpha,position,vel_max

    rospy.init_node('omni_drive_move',anonymous=False)

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
        
    rate = rospy.Rate(200)

    while not rospy.is_shutdown():
        err_pos_y = des_pos_y - position.y
        err_pos_x = des_pos_x - position.x
        robot_err_x =   np.cos(yaw) * err_pos_x + np.sin(yaw) * err_pos_y
        robot_err_y =   - np.sin(yaw) * err_pos_x + np.cos(yaw) * err_pos_y
        
    #check the project readme for more explanation about the equations used in the code bellow
        alpha = math.atan2(err_pos_y, err_pos_x)
        alpha=normalize_angle(alpha)
       

        distance = math.sqrt(pow(des_pos_y + position.y, 2) + pow(des_pos_x + position.x, 2))
        twist_msg = Twist()
        
        
        if math.fabs(err_pos_x) > dist_precision :
            twist_msg.linear.x = vel_max* np.cos(alpha) 
            
        else: 
            twist_msg.linear.x=0 


        if math.fabs(err_pos_y) > dist_precision :
            twist_msg.linear.y = -vel_max*np.sin(alpha) if err_pos_y > 0 else  -vel_max* np.sin(alpha) 
        else: 
            twist_msg.linear.y = 0
            
        pub.publish(twist_msg)
        rate.sleep()

if __name__ == '__main__':
    go(float(sys.argv[1]),float(sys.argv[2]))


