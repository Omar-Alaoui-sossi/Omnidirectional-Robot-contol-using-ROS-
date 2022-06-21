# Omnidirectional-Robot-contol-using-ROS-

   ![ros](https://user-images.githubusercontent.com/91972670/174807777-7a61a37c-355c-4f28-8c5e-d716ecca9398.jpg)

# ROS Package Instalation: 

git clone https://github.com/Omar-Alaoui-sossi/Omnidirectional-Robot-contol-using-ROS-.git

this project is created using ROS Noetic running in an Ubuntu 20.04 ,so if you are running another ROS or Ubuntu distribution you may need to adjust some parameters and configuration .

# Description :

this project is related to the Robocup international challenge where the aim is the to creat a team of 5 midlesized robot to play a football game,in this github repository I am starting from scratch and I will keep uploading and boosting this project till the end of my academical studies .
Untill now I managed to finish the conception phase and the control part of the robot's motion,all what is listed previously is coded using xml,yaml,python and bash. 
All the scripts provided are well commented and i will try to explained it better in the following sections of this readme file .

# Robot Conception:

In order to creat a robot i used the Urdf syntaxe using XML coding language ,it seems hard in the begining but it become easier and faster to code afterwards.
so for better understanding check the photo bellow: 

![image](https://user-images.githubusercontent.com/91972670/174816082-ed457922-fb27-4344-b72e-b7fde29c23d8.png)

this the base code that will be getting repeated over and over to creat the whole structure:

![po](https://user-images.githubusercontent.com/91972670/174818882-c24351d8-38d6-4045-bdd5-88d5cbae8ff4.png)
![image](https://user-images.githubusercontent.com/91972670/174817212-23c12200-475f-4413-95f1-e17c7cebdbe5.png)

for more information check the link right here: http://wiki.ros.org/urdf/Tutorials/Building%20a%20Visual%20Robot%20Model%20with%20URDF%20from%20Scratch

#Inverse Kinematics:

in this section i will explain the inverse kinematics computation I adopted to write my python node that executes the inverse kinematics which is a conversion from a robot motion into wheel velocity .The input of forward kinematics regards the wheels’speed, and the output regards the robot’s velocities 
 to simulate the robot's motion. Converting the four wheels’ speed into the robot’s linear velocity requires the Jacobian matrix . 
![robot](https://user-images.githubusercontent.com/91972670/174830808-97bcdd66-9d69-4114-8dfd-92ae6f779585.PNG)
for the forawads kinematics the equation looks like the following :
![fk](https://user-images.githubusercontent.com/91972670/174830621-9d3207a2-8412-405a-bef7-8d2603910b15.PNG)
and by inversing this matrix we achieve the inverse kinematics :
![ik](https://user-images.githubusercontent.com/91972670/174830730-cf24af80-af49-4b10-83bb-2a9dc060c8b3.png)

