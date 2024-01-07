import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2
from std_msgs.msg import Float64MultiArray
x = 0.0
y = 0.0 
theta = 0.0

def newOdom(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

rospy.init_node("speed_controller")

sub = rospy.Subscriber('/Robot_Arm/Robot_arm_diffdrive_controller/odom', Odometry, newOdom)
pub = rospy.Publisher('/Robot_Arm/Robot_arm_diffdrive_controller/cmd_vel', Twist, queue_size = 1)
pub5 = rospy.Publisher('Robot_Arm/Gripper_controller/command',Float64MultiArray,queue_size=2)
r = rospy.Rate(4)
speed = Twist()
goal = Point()
goal.x = 1.62
goal.y = 0
gripper=Float64MultiArray()
gripper.data=['5.0','5.0']
pub5.publish(gripper)
#gripper.data=[0.0,0.0]
#pub5.publish(gripper)
while not rospy.is_shutdown():
    inc_x = goal.x -x
    inc_y = goal.y -y

    angle_to_goal = atan2(inc_y, inc_x)
    if(inc_x<0):
    	speed.linear.x = 0.0
    	speed.angular.z = 0.0
    else:
        speed.linear.x = 0.2
        speed.angular.z = 0.0

    pub.publish(speed)
    r.sleep()
   
