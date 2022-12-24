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
x = []
y = []
z = []
#link lengths
a1=0.08
a2=0.08
a3=0.08
a4=0.08
#joint variables
q1_1 = Symbol('theta1')
q2_1 = Symbol('theta2')
q3_1 = Symbol('theta3')
q4_1 = Symbol('theta4')
q5_1 = Symbol('theta5')

T1 = Transformation(0,q1_1,0,a1)        #Creating homogeneous Transformation matrix for 0-1
T2 = Transformation(0,q2_1,0,a2)           #Creating homogeneous Transformation matrix for 1-2
T3 = Transformation(pi/2,q3_1,0,a3)     #Creating homogeneous Transformation matrix for 2-3
T4 = Transformation(0,q4_1,0,a4)       #Creating homogeneous Transformation matrix for 3-4
T5 = Transformation(0,q5_1,0,0)       #Creating homogeneous Transformation matrix for 4-5

H1 = T1
H2 = H1*T2
H3 = H2*T3
H4=  H3*T4
H5 = H4*T5



Z0 = Matrix([[0], [0], [1]])                # Z component of base frame
Z1 = Matrix([[H1[2]], [H1[6]], [H1[10]]])   #Taking Z component of H1
Z2 = Matrix([[H2[2]], [H2[6]], [H2[10]]])   #Taking Z component of H2
Z3 = Matrix([[H3[2]], [H3[6]], [H3[10]]])   #Taking Z component of H3
Z4 = Matrix([[H4[2]], [H4[6]], [H4[10]]])   #Taking Z component of H4
Z5 = Matrix([[H5[2]], [H5[6]], [H5[10]]])   #Taking Z component of H5
Xp = Matrix([[H5[3]], [H5[7]], [H5[11]]])   #Translation component of final transformation matrix - H6

C1 = diff(Xp, q1_1) #Differentitaing Xp w.r.t theta 1
C2 = diff(Xp, q2_1) #Differentitaing Xp w.r.t theta 2
C3 = diff(Xp, q3_1) #Differentitaing Xp w.r.t theta 3
C4 = diff(Xp, q4_1) #Differentitaing Xp w.r.t theta 4
C5 = diff(Xp, q5_1) #Differentitaing Xp w.r.t theta 5
J1 = Matrix([[C1], [Z1]]) #Computing Jacobian 1st column
J2 = Matrix([[C2], [Z2]]) #Computing Jacobian 2nd column
J3 = Matrix([[C3], [Z3]]) #Computing Jacobian 3rd column
J4 = Matrix([[C4], [Z4]]) #Computing Jacobian 4th column
J5 = Matrix([[C5], [Z5]]) #Computing Jacobian 5th column
J = Matrix([[J1, J2, J3, J4, J5]])
B1=Matrix([[0.461409], [-0.008111], [0.088838],[1]])
rospy.init_node('send_joints')
pub1 = rospy.Publisher('/Robot_Arm/Shoulder_pan_position_controller/command',
                          Float64, queue_size=1)
pub2 = rospy.Publisher('/Robot_Arm/Shoulder_roll_position_controller/command',
                          Float64, queue_size=1)
pub3 = rospy.Publisher('/Robot_Arm/Elbow_pan_position_controller/command',
                          Float64, queue_size=1)
pub4 = rospy.Publisher('/Robot_Arm/Elbow_pitch_position_controller/command',
                          Float64, queue_size=1)
pub5 = rospy.Publisher('Robot_Arm/Gripper_controller/command',Float64MultiArray,queue_size=1)
pub6=rospy.Publisher('/Robot_Arm/Gripper_pan_position_controller/command',
                          Float64, queue_size=1)
pub7=rospy.Publisher('/Robot_Arm/Gripper_pitch_position_controller/command',
                          Float64, queue_size=1)  
       
D1=B1-H5.col(3)
q1=0
q2=0
q3=pi/2
q4=0
q5=0
D1_1=D1.evalf(subs={q1_1:q1,q2_1:q2,q3_1:q3,q4_1:q4,q5_1:q5})
t=0
while t!=3: 
	D_new=Matrix([[D1_1[0]],[D1_1[1]],[D1_1[2]],[0],[0],[0]])
	D1_1=D1.evalf(subs={q1_1:q1,q2_1:q2,q3_1:q3,q4_1:q4,q5_1:q5})
	J_1=J.evalf(subs={q1_1:q1,q2_1:q2,q3_1:q3,q4_1:q4,q5_1:q5})
	H5_1=H5.evalf(subs={q1_1:q1,q2_1:q2,q3_1:q3,q4_1:q4,q5_1:q5})
	V1=J_1.pinv()*D_new
	V1[0]=V1[0]/V1[1]
	V1[1]=V1[1]/V1[1]
	V1[2]=V1[2]/V1[1]
	V1[3]=V1[3]/V1[1]
	V1[4]=V1[4]/V1[1]
	#pub1.publish(V1[0])
	pub2.publish(V1[1])
	pub3.publish(V1[2])
	pub4.publish(V1[3])
	pub6.publish(V1[4])
	
	q1=q1+V1[0]*0.1
	q2=q2+V1[1]*0.1
	q3=q3+V1[2]*0.1
	q4=q4+V1[3]*0.1
	q5=q5+V1[4]*0.1
	q1=q1.evalf()
	q2=q2.evalf()
	q3=q3.evalf()
	q4=q4.evalf()
	q5=q5.evalf()
	t=t+1
print("Done")	
num=Array(range(10),(2,12))
gripper=Float64MultiArray()
gripper.data=[50.0,50.0]
pub5.publish(gripper) 
pub6.publish(2)
pub7.publish(-1)
print("Done")	
