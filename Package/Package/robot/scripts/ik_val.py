import rospy
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
import sys, select, termios, tty
from sympy import * 
from sympy.matrices import Matrix
from sympy import pprint, Array
import numpy as np
from math import pi
from sympy import diff,cos,sin,Symbol
import math
#Function to create a Transformation matrix using DH parameters - α, θ, d, a
def Transformation(alpha, theta, d, a):
   d = float(d)           
   a = float(a)
   T = Matrix([[cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta)],
   [sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)],
   [0, sin(alpha), cos(alpha), d],
   [0, 0, 0, 1]])
   return T
theta = np.linspace(-180,240, num=50) 
dt = 5/50   #dt = T/N where T is 5 secs and No.of intervals is 50
x = []
y = []
z = []
#joint variables
q1_1=  Symbol('theta1')
q2_1 = Symbol('theta2')
q3_1 = Symbol('theta3')
q4_1 = Symbol('theta4')
a1=0.08
T1 = Transformation(-pi/2,q1_1,0,0)         #Creating homogeneous Transformation matrix for 0-1,fixing q1 to 0
T2 = Transformation(pi/2,q2_1,0,0)           #Creating homogeneous Transformation matrix for 1-d
T3 = Transformation(pi/2,q3_1,2*a1,0)        #Creating homogeneous Transformation matrix for d-2
T4 = Transformation(-pi/2,q4_1,0,0)
H1 = T1
H2 = H1*T2
H3 = H2*T3
H4 = H3*T4
Z0 = Matrix([[0], [0], [1]])                # Z component of base frame
Z1 = Matrix([[H1[2]], [H1[6]], [H1[10]]])   #Taking Z component of H1
Z2 = Matrix([[H2[2]], [H2[6]], [H2[10]]])   #Taking Z component of H2
Z3 = Matrix([[H3[2]], [H3[6]], [H3[10]]])   #Taking Z component of H3
Z4 = Matrix([[H4[2]], [H4[6]], [H4[10]]])   #Taking Z component of H4
Xp = Matrix([[H4[3]], [H4[7]], [H4[11]]])   #Translation component of final transformation matrix - H4
C1 = diff(Xp, q1_1) #Differentitaing Xp w.r.t theta 2
C2 = diff(Xp, q2_1) #Differentitaing Xp w.r.t theta 3
C3 = diff(Xp, q3_1) #Differentitaing Xp w.r.t theta 4
C4=  diff(Xp, q4_1)
J1 = Matrix([[C1], [Z1]]) #Computing Jacobian 1st column
J2 = Matrix([[C2], [Z2]]) #Computing Jacobian 2nd column
J3 = Matrix([[C3], [Z3]])
J4 = Matrix([[C4], [Z4]])
J = Matrix([[J1, J2, J3, J4]])
rospy.init_node('send_joints')
pub1 = rospy.Publisher('/Robot_Arm/Shoulder_pan_position_controller/command', Float64, queue_size=1)
pub2 = rospy.Publisher('/Robot_Arm/Shoulder_roll_position_controller/command', Float64, queue_size=1)
pub3 = rospy.Publisher('/Robot_Arm/Elbow_pan_position_controller/command',Float64, queue_size=1)
pub4 = rospy.Publisher('/Robot_Arm/Elbow_pitch_position_controller/command', Float64, queue_size=1)

#Initial Joint angles
q1 = 0
q2 = 0    #Theta 2
q3 = -pi/2  #Theta 4
q4 = 0
 
# Substituting theta values in Final Trasnformation Matrix H6
for t in theta:
   Vx = 0.1*2*(pi/5)*cos(t*(pi/180))                          # X-component of velocity trajectory
   Vy =  0 # Y-component of velocity trajectory
   Vz = -0.1*2*(pi/5)*sin(t*(pi/180))   # Z-component of velocity trajectory
   Wx = 0                              # Angular velocities are zero
   Wy = 0
   Wz = 0
   X_dot = Matrix([[Vx], [Vy], [Vz], [Wx], [Wy], [Wz]]) # Cartesian v
   J_1 = J.evalf(subs ={q1_1 : q1, q2_1 : q2, q3_1 : q3, q4_1 : q4})
   pprint(J_1)# Substituting theta values in Jacobian Matrix
   H4_1 = H4.subs({q1_1 : q1, q2_1 : q2, q3_1 : q3, q4_1 : q4}) 
   q_dot = J_1.pinv()*X_dot    # Obtaining the joint angular velocities Matrix
   q1_dot = q_dot[0]
   q2_dot = q_dot[1]
   q3_dot = q_dot[2]
   q4_dot = q_dot[3]
   pub1.publish(q1_dot)
   pub2.publish(q2_dot)
   pub3.publish(q3_dot)
   #pub4.publish(q4_dot)
   # Performing numerical integration on the joint angular velocities to obtain the new joint angles
   q1=  q1+  (q1_dot*dt)
   q2 = q2 + (q2_dot*dt)
   q3 = q3 + (q3_dot*dt)
   #q4 = q4 + (q4_dot*dt)
   q1 = q1.evalf()
   q2 = q2.evalf()
   q3 = q3.evalf()
   #q4 = q4.evalf()
  
 


