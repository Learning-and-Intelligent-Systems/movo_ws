#!/usr/bin/env python
# Author: Silvia Knappe
# Control movo with keyboard teleop, adapted from ROS teleop_twist_keyboard.py

#TODO Add instructions "wasd to move" etc

import rospy

from geometry_msgs.msg import Twist
from movo.system_defines import TRACTOR_REQUEST
from movo_msgs.msg import ConfigCmd

import sys, select, termios, tty, threading

class KeyboardTeleop:
    def __init__(self):
        self.msg = """
            Use the keyboard to move around!
            --------------------------------
                    W
                A       D
                    S

                to translate
                ------------

                Q       E

                to rotate
            --------------------------------
            """

        self.moves = {
            'w':[1,0,0],
            'a':[0,1,0],
            's':[-1,0,0],
            'd':[0,-1,0],
            'q':[0,0,1],
            'e':[0,0,-1]
        }

        self.pub = rospy.Publisher('/movo/cmd_vel', Twist, queue_size=1)
        self.cfg_cmd = ConfigCmd()
        self.cfg_pub = rospy.Publisher('/movo/gp_command', ConfigCmd, queue_size=1)
        self.r = rospy.Rate(10)

        self.settings = termios.tcgetattr(sys.stdin)


    def getKey(self):

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

    def setGP(self):
        while not rospy.is_shutdown():
            self.cfg_cmd.gp_cmd = 'GENERAL_PURPOSE_CMD_SET_OPERATIONAL_MODE'
            self.cfg_cmd.gp_param = TRACTOR_REQUEST
            self.cfg_cmd.header.stamp = rospy.get_rostime()
            self.cfg_pub.publish(cfg_cmd)
            self.r.sleep

    def start(self):
        print self.msg

        threading.Thread(target=self.setGP)
        
        while not rospy.is_shutdown():
            try:

                twist = Twist()

                key = self.getKey()

                v_x = 0
                v_y = 0
                a_z = 0

                if key in self.moves:
                    v_x = self.moves[key][0]
                    v_y = self.moves[key][1]
                    a_z = self.moves[key][2]

                else:
                    v_x = 0
                    v_y = 0
                    a_z = 0
                    if (key == '\x03'):
                        break
                        
                twist.linear.x = v_x
                twist.linear.y = v_y
                twist.angular.z = a_z
                self.pub.publish(twist)

                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

            except Exception as e:
                print(e)


if __name__ == "__main__":
    rospy.init_node('keyboard_teleop')
    kt = KeyboardTeleop()
    kt.start()

