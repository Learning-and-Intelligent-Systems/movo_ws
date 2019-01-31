#!/usr/bin/env python
# Author: Silvia Knappe
# Control movo with keyboard teleop, adapted from ROS teleop_twist_keyboard.py

#TODO Add instructions "wasd to move" etc

import rospy

from geometry_msgs.msg import Twist
from movo.system_defines import TRACTOR_REQUEST
from movo_msgs.msg import ConfigCmd

import sys, select, termios, tty

msg = """
Use the keyboard to move around!
--------------------------------
        W
    A       S
        D

    to translate
    ------------

    Q       E

    to rotate
--------------------------------
"""

print msg

moves = {
    'w':[1,0,0],
    'a':[0,1,0],
    's':[-1,0,0],
    'd':[0,-1,0],
    'q':[0,0,1],
    'e':[0,0,-1]
}

def getKey():

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)

    try:
        tty.setraw(sys.stdin.fileno())
        [i,o,e] = select.select([sys.stdin.fileno()],[],[],0.2)
        if i:
            ch = sys.stdin.read(1)
        else:
            ch = None
    finally:
        termios.tcsetattr(fd,termios.TCSADRAIN,old)

    return ch


if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('keyboard_teleop')
    cfg_cmd = ConfigCmd()
    cfg_pub = rospy.Publisher('/movo/gp_command', ConfigCmd, queue_size=1)
    pub = rospy.Publisher('/movo/cmd_vel', Twist, queue_size=1)

    v_x = 0
    v_y = 0
    a_z = 0
    
    try:
        while(1):
            cfg_cmd.gp_cmd = 'GENERAL_PURPOSE_CMD_SET_OPERATIONAL_MODE'
            cfg_cmd.gp_param = TRACTOR_REQUEST
            cfg_cmd.header.stamp = rospy.get_rostime()

            cfg_pub.publish(cfg_cmd)

            twist = Twist()

            key = getKey()

            v_x = 0
            v_y = 0
            a_z = 0

            if key in moves:
                v_x = moves[key][0]
                v_y = moves[key][1]
                a_z = moves[key][2]

            else:
                v_x = 0
                v_y = 0
                a_z = 0
                if (key == '\x03'):
                    break
                    
            twist.linear.x = v_x
            twist.linear.y = v_y
            twist.angular.z = a_z
            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.angular.z = 0
        pub.publish(twist)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
