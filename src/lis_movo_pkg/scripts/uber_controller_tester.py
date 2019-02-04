#!/usr/bin/env python
#Author: Ariel Anders
from lis_movo_pkg.uber_controller import Uber
import rospy

def test_gripper():
    rospy.loginfo("testing open gripper commands")
    uc.close_gripper('l')
    uc.close_gripper('r')
    rospy.loginfo("grippers should be closed")
    uc.open_gripper('l')
    uc.open_gripper('r')
    rospy.loginfo("grippers should be open")

def test_head():
    rospy.loginfo("testing head command")
    uc.look_down_center()
    raw_input("look up")
    uc.look_forward()

def test_torso():
    rospy.loginfo("testing torso command")
    uc.lift_torso()
    raw_input("move torso down")
    uc.down_torso()

def test_joint():
    rospy.loginfo("testing joint control")
    uc.move_arm_to_side("l")
    uc.move_arm_to_side("r")


def test_get_state():
    print "testing gathering state information"
    raw_input("get joint angles-- left")
    print uc.get_joint_positions('l')

    raw_input("get joint angles-- right")
    print uc.get_joint_positions('r')

    raw_input("get cartesian pose-- left")
    print uc.return_cartesian_pose('l', 'base_link')

    raw_input("get cartesian pose--right")
    print uc.return_cartesian_pose('r', 'base_link')

def test_custom_joint():
    rospy.loginfo("sending joints to 0!")
    uc.command_joint_pose('l', [0]*7, time=2, blocking=False)
    uc.command_joint_pose('r', [0]*7, time=2, blocking=True)



rospy.init_node("ubertest")
rospy.loginfo("how to use uber controller")
uc = Uber()
#test_custom_joint()
#test_head() 
#test_torso()
#test_gripper()
#test_joint()
test_get_state()
