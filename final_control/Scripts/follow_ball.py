#!/usr/bin/env python3
__author__="Omar ALAOUI SOSSI"

import sys
import math
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
import numpy as np 


"""
Ball_tracker class 
General description: openCV-based image processing for detecting spherical objects
"""
class Ball_tracker:
    def __init__(self):
        #publisher to velocity topic 
        self.pub=rospy.Publisher('/cmd_vel',Twist,queue_size=10)
        self.rate=rospy.Rate(1)
        self.rot=Twist()

        self.cx=0
        self.cy=0
        self.velx=3 #max vel linear x
        self.velz=2.5 #max vel angular z

        self.bridge = CvBridge()
        self.image_received = False

        # Connect image topic
        img_topic = "/final/camera1/image_raw"
        
        self.image_sub = rospy.Subscriber(img_topic, Image, self.callback)
        self.aim_bot = 0
        self.dpos = [0,0]

        # Allow up to one second to connection
        rospy.sleep(1)
        
    def callback(self, data):

        """
        gets the camera msg from the topic / subscriber call back 
        returns nothing 
        """
        # Convert image to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")        
        self.image_received = True
        self.image = cv_image
        self.find_object(cv_image)


    def find_object(self,img):
        """
        gets the image from the robot's camera which is converted to opencv image
        type=opencv img
        returns and displays an image with a mask frame an image with a rectangle drawn arround the orange object detected 
        also changes the state of the flag that alloows robot movements

        """
        #conversion from bgr to hsv 
        hsv_frame = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        #detemining the lightest orange possibl and the darkest orange possibl 
        lower = np.array([5, 50, 50])
        upper = np.array([15, 255, 255])
        #flag for robot movements activation 
        flag = False
        #mask using the orange color ranges 
        mask_frame=cv2.inRange(hsv_frame,lower,upper)
        #displaying the mask 
        cv2.imshow("mask",mask_frame)
        #contours drawing on each object in that color range
        contours,hierarchy= cv2.findContours(mask_frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        X,Y,W,H=0,0,0,0
        point=(int(self.image.shape[0]/2),int(self.image.shape[1]*3/5))
        #determining a rectangle on each object countoured 
        for pic, contour in enumerate(contours):
            flag = True
            area = cv2.contourArea(contour)
            #the object should has a big area in order to be considered 
            if(area > 30):
                x, y, w, h = cv2.boundingRect(contour)
                if(w*h>W*H):
                    X, Y, W, H= x, y, w, h #rectangle parametres 
            #deteremination of the centre of the rectangle set by the boudingRect method      
            cx = X + W/2
            cy = Y + H/2
            #determination of the distaance between the point given earlier and the position of the ball in pixels 
            self.dpos[0] =  (cx - point[0])
            self.dpos[1] = -(cy - point[1]) #always take in consideration the heading of the height in this case it should be reversed 
        self.move_to_object(flag)
        #rectangle drawing and display in the img frame 
        img = cv2.rectangle(img, (X, Y),(X +W, Y + H),(0, 0, 255), 2)
        #point diplay 
        cv2.circle(img,point,4,(0,255,0),-1)
        #img display with point and the rectangle that contains the orange object 
        cv2.imshow("window", img)
        cv2.waitKey(3)

    def move_to_object(self, flag):
        """
        takes a flag that should be True in order to proceed robots movements 
        type=boolean ,this flag is determined by find object methode
        """
        #since we are working with pixels I add an error margin that can be configured and optimized later on 
        margin = 5
        if flag:
            if self.dpos[0] < -margin or self.dpos[0] > margin:
                #the difference between those two parameters should be normalized since it should not surpass the img dimension (it is in pixels) dpos[0] is the x distance  
                ndif = abs(self.dpos[0] - margin)/(self.image.shape[0]/2)
                prop = math.log(1 + 5*ndif)/math.e #division over math.e is obligatory since we do not desire a velocity that surpace the one set "vel"          
                vel = self.velz*prop #each time we loop we multiply by the prop (logarithmic funtion shifted by one )in order to reduce the velocity each time we got closer to the ball
                print("rotation speed: %5.3f" % vel)
                if self.dpos[0] < -margin:
                    self.rot.angular.z= -vel
                    text="rotating left"
                elif self.dpos[0] > margin:
                    self.rot.angular.z= vel
                    text="rotating right"    
            else:
                self.rot.angular.z=0
                text="not rotating"

            #same logic applied in the y pixel axe 
            if self.dpos[1] < -margin or self.dpos[1] > margin:
                ndif = abs(self.dpos[1] - margin)/(self.image.shape[1]/2) #computes the difference between the y distance and the error margin in pixels
                prop = math.log(1 + 100*ndif)/math.e
                vel = self.velx*prop
                print("translation speed: %5.3f" % vel)
                if self.dpos[1] < -margin:
                    self.rot.linear.x= -vel
                    text+=", translating backwards"
                elif self.dpos[1] > margin:
                    self.rot.linear.x= vel
                    text+=", translating forward"
                
            else:
                self.rot.linear.x=0
                text+=", not translating"

        else:
            self.rot.angular.z=self.velz
            self.rot.linear.x=0
            text="seeking ball........."

        print(text)
        self.pub.publish(self.rot)
      
    #stoping the robt by publishing nothing     
    def stop(self):
        self.rot.angular.z=0
        self.rot.linear.x=0
        self.pub.publish(self.rot)
        cv2.destroyAllWindows()
        print('######## stop #########')



def main():
    rospy.init_node('ball_tracker', anonymous=False)
    script = Ball_tracker()

    rospy.on_shutdown(script.stop) # stoping all robot motions topics img frames after Ctrl - C

    while not rospy.is_shutdown():
        rospy.sleep(0.1)
        rospy.spin()


if __name__ == '__main__':

    main()
