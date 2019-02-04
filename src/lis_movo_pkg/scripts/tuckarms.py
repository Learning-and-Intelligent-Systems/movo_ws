#!/usr/bin/env python

import sys, rospy
from std_msgs.msg import Bool
from moveit_python import MoveGroupInterface
from moveit_msgs.msg import MoveItErrorCodes
from movo_action_clients.gripper_action_client import GripperActionClient
import moveit_commander
from argparse import ArgumentParser

class MovoUpperBody:
    upper_body_joints = [
        "right_shoulder_pan_joint",
        "right_shoulder_lift_joint",
        "right_arm_half_joint",
        "right_elbow_joint",
        "right_wrist_spherical_1_joint",
        "right_wrist_spherical_2_joint",
        "right_wrist_3_joint",
        "left_shoulder_pan_joint",
        "left_shoulder_lift_joint",
        "left_arm_half_joint",
        "left_elbow_joint",
        "left_wrist_spherical_1_joint",
        "left_wrist_spherical_2_joint",
        "left_wrist_3_joint",
        "linear_joint",
        "pan_joint",
        "tilt_joint"]

    homed = [-1.5,-0.2,-0.175,-2.0,2.0,-1.24,-1.1, \
            1.5,0.2,0.15,2.0,-2.0,1.24,1.1,0.35,0,0]
    tucked = [-1.6,-1.4,0.4,-2.7,0.0,0.5,-1.7, 1.6,1.4,\
            -0.4,2.7,0.0,-0.5, 1.7, 0.04, 0, 0]


    gripper_closed = 0.01
    gripper_open = 0.165

    def __init__(self):
        self.move_group = MoveGroupInterface("upper_body","base_link")
        self.move_group.setPlannerId("RRTConnectkConfigDefault")
        self.lgripper = GripperActionClient('left')
        self.rgripper = GripperActionClient('right')



    def untuck(self):
        rospy.loginfo("untucking the arms")

        self.lgripper.command(self.gripper_open)
	self.rgripper.command(self.gripper_open)
	success=False
	while not rospy.is_shutdown() and not success:	
	    result = self.move_group.moveToJointPosition(\
                    self.upper_body_joints, self.homed, 0.05)
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                    success = True
            else:
                    rospy.logerr("moveToJointPosition failed (%d)"\
                            %result.error_code.val)
    def tuck(self):
	success=False
	while not rospy.is_shutdown() and not success:
            result = self.move_group.moveToJointPosition(\
                    self.upper_body_joints, self.tucked, 0.05)
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                    success = True

	self.lgripper.command(self.gripper_closed)
	self.rgripper.command(self.gripper_closed)

if __name__ == "__main__":
    parser = ArgumentParser(description="Tuck or untuck the movo arms")
    parser.add_argument("-u", "--untuck", \
            help="untuck the arms", action='store_true')
    parser.add_argument("-t", "--tuck", \
            help="tuck the arms", action='store_true')


    args = parser.parse_args()
    
    rospy.init_node('tuckarms')
    mub = MovoUpperBody()
    if args.tuck:
        mub.tuck()
    if args.untuck:
        mub.untuck()
