# Mobile-Manipulator-Robot-modeling-and-simulation-using-Gazebo-ROS-Noetic-
Performed Pick and place operation using the mobile manipulator in Gazebo. Application - cleaning in compact and cluttered environments

## Table of Contents
1. [Introduction](#1-introduction)
2. [Process](#2-process)
3. [Robot Description](#3-robot-description)
4. [Forward Kinematics](#4-forward-kinematics)
   - [4.1 D-H Parameters](#41-d-h-parameters)
5. [Inverse Kinematics](#5-inverse-kinematics)
6. [Forward Kinematics Validation](#6-forward-kinematics-validation)
7. [Inverse Kinematics Validation](#7-inverse-kinematics-validation)
8. [Workspace Study](#8-workspace-study)
9. [Assumptions](#9-assumptions)
10. [Control Method](#10-control-method)
11. [Gazebo and RViz Visualization](#11-gazebo-and-rviz-visualization)
12. [Problems Faced](#12-problems-faced)
13. [Conclusion](#14-conclusion)
14. [References](#16-references)

## 1. Introduction
Modeling a mobile robot (differential drive) with a robotic arm named CR-7. The goal is to create a versatile robot capable of cleaning spaces efficiently.

![CR-7 in Gazebo](link-to-image)

## 2. Process
The project involved a series of analyses and processes, including the study of robot kinematics (inverse and forward), validation, and simulation in Gazebo and RViz. The derived equations were intended for practical applications such as goal navigation, object manipulation, and space cleaning.
### Project Pipeline
![Process](link-to-image)

## 3. Robot Description
Robot Type: MOBILE MANIPULATOR
Total DOF (9) = Robotic arm DoF (6) + Mobile robot DoF (3)

#### DIMENSIONS:
- Chassis property(units): Length: 0.19, Width: 0.19, Height: 0.070
- Wheel property: Radius(units):0.055, width: 0.022, Mass=0.2
- Shoulder roll link(units): Length: 0.09, Width: 0.047, Height: 0.035
- Shoulder pan link(units): Length: 0.09, Width: 0.047, Height: 0.035
- Elbow pan Link(units): Length: 0.09, Width: 0.047, Height: 0.035
- Elbow pitch Link(units): Length: 0.09, Width: 0.047, Height: 0.035
- Gripper(units): Length: 0.05, Width: 0.010
- Gripper type: 2 Finger gripper


## 6. Forward Kinematics Validation
To validate forward kinematics, a video of the robot's movement in Gazebo and RViz is available Below
![Forward Kinematics Validation](link-to-video)

## 7. Inverse Kinematics Validation
To validate inverse kinematics, a video of the robot's movement in Gazebo and RViz is available below 
[Inverse Kinematics Validation](link-to-video).

## 8. Workspace Study
![Dexterous Workspace](link-to-image)

Maximum workspace of the arm is as follows:
The above figure is obtained by selecting the angle limits as follows:
- Theta 1= 0:2*pi
- Theta 2= -pi/2:pi/2
- Theta 3=0:pi/2
- Theta 4= -pi/2:pi/2

## 9. Assumptions
- All the joints and objects are considered to be rigid.
- The friction and other external disturbances are not taken into
account.
- Robot self-collision is considered along with the external
obstacles and is taken into account for this scope of the project.
- The path of the arm or the robot is just one solution among all
the other solutions it can have, this may or may not be the
optimal solution.

## 10. Control Method
The differential drive mobile robot operates through a closed-loop mechanism, as outlined below:

1. **Predefined Goal:** The robot's destination is specified in the code.

2. **Fixed Starting Position:** The initial position of the robot is predetermined.

3. **Control Inputs:** Wheel control inputs are provided, and the robot's current status is monitored using the Gazebo odometer.

4. **Distance Calculation:** The current distance to the goal is computed, and control adjustments are made based on linear and angular velocity.

5. **Precise Stopping:** The robot halts 0.25 units before reaching the target location for efficient pick-up.

6. **Return Capability:** The robot can also autonomously return to its original or home position.

## 11. Gazebo and RViz Visualization

For robot movement validation, the following videos are available:
- Teleop: [link-to-teleop-video]
- Arm Control: [link-to-arm-control-video]
- Closed Loop Control: [link-to-closed-loop-video]
- RQT Steering: [link-to-rqt-steering-video]

For pick and place operation, refer to the video [here](link-to-video).

## 12. Problems Faced
Robot Spawning Issues: The robot was spawning with dislocated parts, leading to incorrect simulations. Debugging and adjusting the URDF file resolved this issue.

DH Parameter Determination: Finding the Denavit-Hartenberg (DH) parameters for the robot was a task that required precision. As the robot used in our project is not universal, identifying the correct parameters was crucial for accurate kinematic analysis.

Simulation Realism: Achieving a realistic simulation proved challenging, especially in terms of adding sensors. Due to constraints, we used a Gazebo odometer instead of sensors for the closed-loop controller.

## 13. Conclusion
In conclusion, the project successfully modeled a cleaning robot, CR-7, integrating a differential drive mobile robot with a robotic arm. The robot's URDF file was accurately spawned in Gazebo and RViz, and kinematic analyses were performed for both forward and inverse kinematics.

The implementation of control methods, such as closed-loop control for the differential drive, showcased the robot's ability to navigate and stop accurately. Despite the challenges faced, the project achieved its objectives and laid the foundation for future advancements.

## 14. References
1. Robot Modeling and Control, Mark W. Spong, Seth Hutchinson, and M. Vidyasagar
2. HW3 or HW4 of ENPM-662
3. Gazebo.sim
4. Ros.org
5. https://moveit.ros.org/
6. https://www.solidworks.com/


Authors: Rishikesh Jadhav(UID:119256534) and Nishant Awdeshkumar Pandey(UID:119247556).

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
 
