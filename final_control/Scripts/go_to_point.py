#! /usr/bin/env python3
__author__="Omar ALAOUI SOSSI"
# this script is treating our omnidirectional robot as a diff driver robot #
################################### neeeds optimisation ###########################################
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *
import sys
import math

#parametres initialization
position_ = Point()
yaw_ = 0
state_ = 0
goal=[-3,-3]
desired_position_ = Point()
desired_position_.x = goal[0] 
desired_position_.y = goal[1]
desired_yaw=0
yaw_precision_ = math.pi / 90 # +/- 2 degree allowed
dist_precision_ = 0.4592568956884
pub = None


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
    yaw_ = euler[2] #robor orientation

def change_state(state):
    """
    flag that helps determining which operation should be executed 
    takes an integer 0,1 or 2   
    """
    global state_
    state_ = state

def move(x_,z_):
    twist_msg = Twist()
    twist_msg.linear.x = x_
    twist_msg.angular.z = z_
    pub.publish(twist_msg)
    
def normalize_angle(angle):#between pi and -pi
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

def fix_yaw(des_pos): #fixing orientattion yaw equal to the goal postion
    """
    takes 2 floats x position and y position 
    return nothing 
"""
    global yaw_, pub, yaw_precision_, state_,desired_yaw
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = normalize_angle(desired_yaw - yaw_)
    print("########## Task in progress : ROTATING ###############")
    
    if math.fabs(err_yaw) > yaw_precision_ :
        if err_yaw > 0:
            move(0.0,-0.5)
        else:
            move(0.0,0.5)
    else :
        move(0.0,0.0)
        change_state(1)

def go_straight_ahead(des_pos): # moving toward the goal
        """
    takes 2 floats x position and y position 
    return nothing 
"""

    global yaw_, pub, yaw_precision_, state_,desired_yaw
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y + position_.y, 2) + pow(des_pos.x + position_.x, 2))
    print("###### Task in progress : moving foraward ######")
    if err_pos > dist_precision_:
        move(1.0,0.0)   
    else:
        move(0.0,0.0)        
        change_state(2)
        print("############# ARRIVED ###########")

def main():
    global pub

    rospy.init_node('go_to_point')

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    
    rate = rospy.Rate(500)

    while not rospy.is_shutdown():
    #since we defined each state for each operation we can use them as flags to activate each function    
        if state_ == 0:
            fix_yaw(desired_position_)
            
        elif state_ == 1:
            go_straight_ahead(desired_position_)
            
        elif state_ == 2:
            move(0.0,0.0)
            
        rate.sleep()

if __name__ == '__main__':
    main()
