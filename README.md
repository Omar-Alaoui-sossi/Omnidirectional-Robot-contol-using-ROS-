# Omnidirectional-Robot-contol-using-ROS-

<p align="center">
   <img src=https://user-images.githubusercontent.com/91972670/174865626-2edb3505-2600-4497-8e95-7cce75de9c61.png >  
</p>

this project is created using ROS Noetic running in an Ubuntu 20.04 ,so if you are running another ROS or Ubuntu distribution you may need to adjust some parameters and configuration .

# ROS Package Instalation: 

Copy paste the following command lines in your terminal in order to set up your work space  :
`mkdir catkin_ws`
`wstool init src `
`catkin_make`
`cd catkin_ws`
``cd src``
``git clone https://github.com/Omar-Alaoui-sossi/Omnidirectional-Robot-contol-using-ROS-.git``
``cd ..``
``catkin_make``

# Simulation :
after setting up the catking workspace 
In order to launch the robot in Gazebo use the following command line :

`roslaunch final_gazebo final.launch `

for rviz simulation :

`roslaunch final_description rviz.launch`


# Description :

this project is related to the Robocup international challenge where the aim is the to creat a team of 5 midlesized robot to play a football game,in this github repository I am starting from scratch and I will keep uploading and boosting this project till the end of my academical studies .
Untill now I managed to finish the conception phase and the control part of the robot's motion,all what is listed previously is coded using xml,yaml,python and bash. 
All the scripts provided are well commented and i will try to explained it better in the following sections of this readme file .

# Robot Conception:

In order to creat a robot i used the Urdf syntaxe using XML coding language ,it seems hard in the begining but it become easier and faster to code afterwards.
so for better understanding check the photo bellow: 
![image](https://user-images.githubusercontent.com/91972670/174873302-d48db0a5-4ba5-4895-b529-403a3714cd42.png)

this the base code that will be getting repeated over and over to creat the whole structure:
<p align="center">
<img src="https://user-images.githubusercontent.com/91972670/174870565-ea2b0fcf-8f25-42a5-a513-d5deab6fc4fe.png" width="500" height="500" />
</p>
<p align="center">
<img src="https://user-images.githubusercontent.com/91972670/174872013-879a91e1-6268-49d0-8600-72370004bf02.png" width="500" height="500" />
</p>
for more information I really recomand to follow this tutorial: http://wiki.ros.org/urdf/Tutorials/Building%20a%20Visual%20Robot%20Model%20with%20URDF%20from%20Scratch

# Inverse Kinematics:

in this section i will explain the inverse kinematics computation I adopted to write my python node that executes the inverse kinematics which is a conversion from a robot motion into wheel velocity .The input of forward kinematics regards the wheels’speed, and the output regards the robot’s velocities 
 to simulate the robot's motion. Converting the four wheels’ speed into the robot’s linear velocity requires the Jacobian matrix . 
 <p align="center">
  <img src=https://user-images.githubusercontent.com/91972670/174830808-97bcdd66-9d69-4114-8dfd-92ae6f779585.PNG >
 </p>

for the forawads kinematics the equation looks like the following :
<p align="center">
   <img src=https://user-images.githubusercontent.com/91972670/174830621-9d3207a2-8412-405a-bef7-8d2603910b15.PNG>
</p>
and by inversing this matrix we achieve the inverse kinematics :
<p align="center">
  <img src=https://user-images.githubusercontent.com/91972670/174830730-cf24af80-af49-4b10-83bb-2a9dc060c8b3.png>
</p>
if you are interseted in the whole calculation prosess I highly recomend you to check this : https://www.researchgate.net/publication/324929202_Motion_Improvement_of_Four-Wheeled_Omnidirectional_Mobile_Robots_for_Indoor_Terrain/fulltext/5aebadf2458515f59981e3b1/Motion-Improvement-of-Four-Wheeled-Omnidirectional-Mobile-Robots-for-Indoor-Terrain.pdf

and : https://www.researchgate.net/publication/308570348_Inverse_kinematic_implementation_of_four-wheels_mecanum_drive_mobile_robot_using_stepper_motors
