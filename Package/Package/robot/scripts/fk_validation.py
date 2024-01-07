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
#joint variables
q1_1 = Symbol('theta1')
q2_1 = Symbol('theta2')
q3_1 = Symbol('theta4')
q4_1 = Symbol('theta5')



T1 = Transformation(-pi/2,q1_1,a1,0)         #Creating homogeneous Transformation matrix for 0-1
T2 = Transformation(pi/2,q2_1,0,0)           #Creating homogeneous Transformation matrix for 1-2
T3 = Transformation(pi/2,q3_1,2*a1,0)        #Creating homogeneous Transformation matrix for 2-3
T4 = Transformation(-pi/2,q4_1,0,0)        #Creating homogeneous Transformation matrix for 3-4


H1 = T1
H2 = H1*T2
H3 = H2*T3
H4=  H3*T4


q1=0
q2=pi/2
q3=0
q4=0

H4_1=H4.evalf(subs={q1_1:q1,q2_1:q2,q3_1:q3,q4_1:q4})

print("CONFIGURATION 1- theta2 = 90 rest all are zero")
pprint(H4_1)
