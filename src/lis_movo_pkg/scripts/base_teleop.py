#!/usr/bin/env python
# Author: Silvia Knappe
# Using a Logitech Wireless Gamepad F710 Controller

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from movo.system_defines import TRACTOR_REQUEST
from movo_msgs.msg import ConfigCmd
import threading


def setGP():
    while not rospy.is_shutdown():
        cfg_cmd.gp_cmd = 'GENERAL_PURPOSE_CMD_SET_OPERATIONAL_MODE'
        cfg_cmd.gp_param = TRACTOR_REQUEST
        cfg_cmd.header.stamp = rospy.get_rostime()
        cfg_pub.publish(cfg_cmd)
        r.sleep()



def callback(joy):
    #cfg_cmd.gp_cmd = 'GENERAL_PURPOSE_CMD_SET_OPERATIONAL_MODE'
    #cfg_cmd.gp_param = TRACTOR_REQUEST
    #cfg_cmd.header.stamp = rospy.get_rostime()

    #cfg_pub.publish(cfg_cmd)

    #rospy.sleep(0.1)
    
    twist = Twist()

    # Button mapping
    deadman = joy.axes[2]
    vel_x = joy.axes[1]
    vel_y = joy.axes[0]
    rotate = joy.axes[3]

    # Use left trigger as deadman: actions only happen if trigger pressed down
    if deadman < -0.5:
        d = 2
    else:
        d = 0

    twist.linear.x = d*vel_x
    twist.linear.y = d*vel_y
    twist.angular.z = d*rotate
    pub.publish(twist)
    #rospy.loginfo("publishing %s " % twist)

def start():
    rospy.init_node('movojoy')
    global pub, sub, cfg_cmd, cfg_pub,r

    r = rospy.Rate(10)
    cfg_cmd = ConfigCmd()
    cfg_pub = rospy.Publisher('/movo/gp_command', ConfigCmd, queue_size=10)

    pub = rospy.Publisher('/movo/cmd_vel', Twist, queue_size=5)
    sub = rospy.Subscriber("joy", Joy, callback, queue_size=1)
    threading.Thread(target=setGP)
    rospy.spin()

if __name__ == '__main__':
    start() 
