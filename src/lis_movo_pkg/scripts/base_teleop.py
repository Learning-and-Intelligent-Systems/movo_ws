#!/usr/bin/env python
# Author: Silvia Knappe
# Using a Logitech Wireless Gamepad F710 Controller

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from movo.system_defines import TRACTOR_REQUEST
from movo_msgs.msg import ConfigCmd

def callback(joy):
    cfg_cmd.gp_cmd = 'GENERAL_PURPOSE_CMD_SET_OPERATIONAL_MODE'
    cfg_cmd.gp_param = TRACTOR_REQUEST
    cfg_cmd.header.stamp = rospy.get_rostime()

    cfg_pub.publish(cfg_cmd)

    rospy.sleep(0.1)
    
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
    global pub, sub, cfg_cmd, cfg_pub
    cfg_cmd = ConfigCmd()
    cfg_pub = rospy.Publisher('/movo/gp_command', ConfigCmd, queue_size=10)
    pub = rospy.Publisher('/movo/cmd_vel', Twist, queue_size=10)
    sub = rospy.Subscriber("joy", Joy, callback, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    start() 
