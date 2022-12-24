# Mobile-Manipulator-Robot-modeling-and-simulation-using-Gazebo-ROS-Noetic-
Performed Pick and place operation using the mobile manipulator in Gazebo. Application - cleaning in compact and cluttered environments

This folder is created for project-2 of ENPM 662 (Introduction to Robot Modelling) by Nishant Awdeshkumar Pandey and Rishikesh Jadhav (UID:119247556,119256534 respectively.

This folder consists of following folders:

1.CAD files - Contains part files (.sldprt), assembly files (.sldasm) of the robot model. 

2.Package - Contains another folder named as robot that can be copy pasted in catkin_ws/src. 

3.Presentation - Presentation given during class

4.Report - Report of the project in .pdf format.

robot folder contains of multiple folders with following description:

1. congif - contains .yaml files for controllers.

2. launch - contains .launch files to spawn the bot in Gazebo, RViz.

3. meshes - contains .dae files for various parts of the bot

4. scripts - contains various .py files to move the bot (publisher, subscriber nodes).

5. urdf - contains .xacro file of the bot

6. worlds - .world in which the bot is spawned in Gazebo

There are various scripts as follows:

1. teleop.py - control the movement of the bot using the teleop

2. fk_validation.py - for forward kinematics validation

3. ik_val.py - for inverse kinematics validation

4. diff_control.py - to control the differential drive of the robot and make it stop just before the goal using closed loop controller

5. go_home.py - to go back to home position of the bot i.e. origin after pick and place operation.

6. grasp.py - to grasp an object using the gripper

7. arm_control.py - to move the arm above the box and open the gripper. (will not work as the size of the object and friction between the gripper and the object is not right just pick and place operation was done manually in the video. Pkease use that as reference for grading).

8. End_eff_trajectory.py - to obtain the graph of end effector trajectory for Inverse kinematics validation.

To spawn the bot in Gazebo and RViz in the world designed the following command works(with rqt_steering) after navigating to the launch folder in terminal:

roslaunch 04-diff_drive_robot_arm.launch

To spawn the bot in RViz with slider to control the various joints of the robot the following command works after navigating to the launch folder in terminal:

roslaunch view_demo.launch
 
To run these codes open in terminal the scripts folder and run python3 <name of the script to run> command along with roscore running in the background.

The following sequence of scripts should be run to obtain the pick and place of an object:

roslaunch 04-diff_drive_robot_arm.launch --> python3 diff_control.py --> python3 arm_control.py --> python3 grasp.py --> go_home.py
 
