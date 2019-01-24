#!/usr/bin/env python
# Author: Silvia Knappe
# Using a Logitech Wireless Gamepad F710 Controller

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

def callback(joy):
    twist = Twist()

    # Button mapping
    deadman = joy.axes[2]
    vel_x = joy.axes[1]
    vel_y = joy.axes[0]
    rotate = joy.axes[3]

    # Use left trigger as deadman: actions only happen if trigger pressed down
    if deadman < -0.8:
        d = 1
    else:
        d = 0

    twist.linear.x = d*vel_x
    twist.linear.y = d*vel_y
    twist.angular.z = d*rotate
    pub.publish(twist)

def start():
    rospy.init_node('movojoy')
    global pub, sub
    pub = rospy.Publisher('/movo/cmd_vel', Twist)
    sub = rospy.Subscriber("joy", Joy, callback)
    rospy.spin()

if __name__ == '__main__':
    start() 
