#!/usr/bin/env python3

import rclpy

from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from math import atan2
import math
from geometry_msgs.msg import Point
n=0
xlist= [4.0,-3.0,7.0,-9.0]
ylist= [7.0,3.0,-2.0,9.0]
def Robotpos(msg):
    global x
    global y
    global theta

    x= msg.pose.pose.position.x
    y= msg.pose.pose.position.y
    rot_q=msg.pose.pose.orientation #lees de positie en orientatie in van de odometry msg
    
    (roll, pitch, theta)=euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

rclpy.init()



node=rclpy.create_node('move')
sub= node.create_subscription(
    Odometry, 'odom', Robotpos, 10
)
pub=node.create_publisher(Point,'goalxy',10)
goal=Point()
goal.x= 0.0
goal.y=0.0
xff=goal.x
r= node.create_rate(4)

while rclpy.ok():
    
    rclpy.spin_once(node)
    inc_x = goal.x - x   #delta x
    inc_y = goal.y - y 
    print("goal:(" + str(goal.x) + "," + str(goal.y) + ")")
    if math.sqrt(inc_x*inc_x + inc_y*inc_y)<0.1: #stop als de robot bij het punt is
        goal.x=xlist[n]
        goal.y=ylist[n]
        n=n+1
    pub.publish(goal)
    r.sleep
