#!/usr/bin/env python
#Author: Ariel Anders
import numpy as np
import rospy
import tf

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
    GripperCommandAction,
    GripperCommandGoal
)

from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import Twist
from actionlib import SimpleActionClient
from sensor_msgs.msg import JointState
from actionlib_msgs.msg import GoalStatus

class UberController:
    simple_clients = {
    'torso':(
        'movo/torso_controller/follow_joint_trajectory',
        FollowJointTrajectoryAction),
    'head':('movo/head_controller/follow_joint_trajectory',
	FollowJointTrajectoryAction),
    'left_gripper': (
        'movo/left_gripper_controller/gripper_cmd', 
        GripperCommandAction),
    'right_gripper': (
        'movo/right_gripper_controller/gripper_cmd', 
        GripperCommandAction),
    'right_joint': (
        'movo/right_arm_controller/follow_joint_trajectory',
        FollowJointTrajectoryAction),
    'left_joint': (
        'movo/left_arm_controller/follow_joint_trajectory',
        FollowJointTrajectoryAction),

    }

    

    """ 
    ============================================================================
                    Initializing all controllers    
    ============================================================================
    """ 


    def __init__(self):
        self.clients = {}
        self.joint_positions = {}
        self.tf_listener = tf.TransformListener()

        rospy.loginfo("starting simple action clients")
        for item in self.simple_clients:
            self.clients[item]= SimpleActionClient(*self.simple_clients[item]) 
            rospy.loginfo("%s client started" % item )
        rospy.sleep(1)
        
        for item in self.clients:
            res = self.clients[item].wait_for_server(rospy.Duration(.1))
            if res:
                rospy.loginfo("%s done initializing" % item )
            else:
                rospy.loginfo("Failed to start %s" % item )
        
            
        self.rate = rospy.Rate(10)
        rospy.loginfo("subscribing to state messages")
        
        self.joint_positons = {}
        self.joint_velocities = {}
        self.joint_sub = rospy.Subscriber(\
                "joint_states", JointState, self.jointCB)
                    
        self.finished_registering()
        rospy.loginfo("done initializing Uber Controller!") 

    """ 
    =============================================================== #XXX make these all nice :)
                    State subscriber callbacks    
    ===============================================================
    """ 

    def finished_registering(self):
        def not_ready():
            return (self.joint_positions.get('linear_joint', None) == None)

        end_time = rospy.Time.now() + rospy.Duration (1)
        test = not_ready()
        while(not rospy.is_shutdown()and rospy.Time.now() <end_time and test):
            self.rate.sleep()
            test = not_ready()
        if not_ready():
            rospy.loginfo("Warning! did not complete subscribing") 
        else:
            rospy.loginfo("finished registering") 
    
    def jointCB(self,data):
        pos = dict(zip(data.name, data.position))
        vel = dict(zip(data.name, data.velocity))
        self.joint_positions = pos
        self.joint_velocities = vel 
    

    def get_arm(self, char):
        if char == "l" or char =="left":
            arm = "left"
        elif char == "r" or char == "right":
            arm = "right"
        return arm

    """ 
    ===============================================================
                   Get State information    
    ===============================================================
    """ 
 
    def get_torso_pose(self):
        return self.joint_positions['linear_joint']

    def get_gripper_pose(self, arm):
        arm = self.get_arm(arm)
        return self.joint_positions['%s_gripper_finger1_joint'%arm]

    def get_head_pose(self):
        head = (
                self.joint_positions['pan_joint'],
                self.joint_positions['tilt_joint'] )
        return head

    def get_joint_positions(self, arm):
        pos = [self.joint_positions[p] for p in self.get_joint_names(arm)]
        return pos
    
    def get_joint_velocities(self, arm):
        vel = [self.joint_velocities[p] for p in self.get_joint_names(arm)]
        return vel

    #return the current Cartesian pose of the gripper
    def return_cartesian_pose(self, arm, frame = 'base_link'):
        end_time = rospy.Time.now() + rospy.Duration(5)
        arm = self.get_arm(arm)
        link = arm + "_gripper_finger1_finger_tip_link"
        while not rospy.is_shutdown() and rospy.Time.now() < end_time:
            try:
                t = self.tf_listener.getLatestCommonTime(frame, link)
                (trans, rot) = self.tf_listener.lookupTransform(frame, link, t)
                return list(trans) , list(rot)
            except (tf.Exception, tf.ExtrapolationException):
                rospy.sleep(0.5)
                current_time = rospy.get_rostime()
                rospy.logerr(\
                "waiting for a tf transform between %s and %s"%\
                (frame, link))
        rospy.logerr("return_cartesian_pose waited 10 seconds tf\
                transform!  Returning None")
        return None, None

    """ 
    ===============================================================
                Send Commands for Action Clients                
    ===============================================================
    """ 
    def send_command(self, client, goal, blocking=False, timeout=None):
        if client == 'head':
            if blocking:
                self.clients[client].send_goal_and_wait(goal)
            else:
                self.clients[client].send_goal(goal)
        else:
            self.clients[client].send_goal(goal)
            rospy.sleep(.1)
            rospy.loginfo("command sent to %s client" % client)
            status = 0
            if blocking: #XXX why isn't this perfect?
                end_time = rospy.Time.now() + rospy.Duration(timeout+ .1)
                while (
                        (not rospy.is_shutdown()) and\
                        (rospy.Time.now() < end_time) and\
                        (status < GoalStatus.SUCCEEDED) and\
                        (type(self.clients[client].action_client.last_status_msg) != type(None))):
                    status = self.clients[client].action_client.last_status_msg.status_list[-1].status #XXX get to 80
                    self.rate.sleep()
                if status >= GoalStatus.SUCCEEDED:
                    rospy.loginfo("goal status achieved.  exiting")
                else:
                    rospy.loginfo("ending due to timeout")

            result = self.clients[client].get_result()
            return result
        return None

    def command_torso(self, pose, time, blocking):
        self.command_joint_pose("torso", [pose], time, blocking)
    
    def command_head(self, angles, time, blocking):
        self.command_joint_pose('head', angles, time, blocking)
    
    def command_gripper(self, arm, position, max_effort, blocking, timeout):
        goal = GripperCommandGoal()
        goal.command.position = position
        goal.command.max_effort = max_effort
        arm = self.get_arm(arm)
        client = "%s_gripper"% arm
        return self.send_command(client, goal, blocking, timeout)
    
    """ 
    ===============================================================
                    Joint Control Commands 
    ===============================================================
    """ 
    # angles is a list of joint angles, times is a list of times from start
    # control group is "right, left, torso, or head"
    def command_joint_trajectory(self, control_group, angles, times, blocking):
        timeout=times[-1] + 1.0

        goal = FollowJointTrajectoryGoal()
        goal.goal_time_tolerance = rospy.Time(0.1)
        goal.trajectory.joint_names =self.get_joint_names(control_group)
       
        # add current point
        point = JointTrajectoryPoint()
        point.positions =self.get_joint_positions(control_group)
        point.velocities = [0.0] * len(goal.trajectory.joint_names)
        point.time_from_start = rospy.Duration(0)
        goal.trajectory.points.append(point)

        for i, (ang, t) in enumerate(zip(angles, times)):
            point = JointTrajectoryPoint()
            point.positions = ang
            point.time_from_start = rospy.Duration(t)
            if i == len(angles) -1:
                point.velocities = [0.0] * len(goal.trajectory.joint_names)
            goal.trajectory.points.append(point)
        goal.trajectory.header.stamp = rospy.Time.now()
        if control_group in ["torso", "head"]:
            client = control_group
        else:
            arm = self.get_arm(control_group)
            if arm is not None:
                client = "%s_joint" % arm

            else:
                rospy.logerr("Control group must be right, left, torso or head.  %s is invalid! " % control_group)
                return
        return self.send_command(client, goal, blocking, timeout)
        #return self.send_command("%s_joint"%arm, goal, blocking, timeout)
    
    # for convience
    def command_joint_pose(self, arm, angles, time, blocking):
        return self.command_joint_trajectory(arm, [angles], [time],  blocking)


    def get_joint_names (self, control_group):
        if control_group == "head":
            return ['pan_joint', 'tilt_joint']
        elif control_group == "torso":
            return ['linear_joint']

        arm = self.get_arm(control_group)
        joints = [
                    '_shoulder_pan_joint', 
                    '_shoulder_lift_joint', 
                    '_arm_half_joint', 
                    '_elbow_joint', 
                    '_wrist_spherical_1_joint', 
                    '_wrist_spherical_2_joint', 
                    '_wrist_3_joint'
                    ]
        return ["%s%s" % (arm, j) for j in joints]

     
""" 
============================================================================
            Uber builds on the UberController with set default values
            for easier use. 
============================================================================
""" 
             
class Uber(UberController):
    timeout = 3

    def freeze(self, arm):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names =self.get_joint_names(arm)
        goal.trajectory.header.stamp = rospy.Time.now()
        return self.send_command("%s_joint"%arm, goal, False)

    def set_timeout(self, time):
        self.timeout = time

    def open_gripper(self, arm, blocking=True):
        self.command_gripper(arm, 0.165, -1.0, blocking=blocking, timeout=self.timeout)
    
    def close_gripper(self,arm, blocking=True):
        self.command_gripper(arm, 1.0, 10, blocking=blocking, timeout=self.timeout)

    def look_down_center(self, blocking=True):
        self.command_head([0,-np.pi/6.], 3, blocking=blocking)

    def look_forward(self, blocking=True):
        self.command_head([0,0], 3, blocking=blocking)

    def lift_torso(self, blocking=True):
        self.command_torso(0.2, blocking=blocking, time=self.timeout)
    
    def down_torso(self, blocking=True):
        self.command_torso(0.1, blocking=blocking, time=self.timeout)

    def move_arm_to_front(self, arm, blocking=True):
        angles = [0.]*7
        self.command_joint_pose(arm, angles, self.timeout, blocking=blocking)

    def move_arm_to_side(self, arm, blocking=True):
        r  = [-1.5,-0.2,-0.15,-2.0,2.0,-1.24,-1.1]
        l =  [1.5,0.2,0.15,2.0,-2.0,1.24,1.1]

        if arm == "l":
            self.command_joint_pose('l', l, self.timeout, blocking=blocking)
        elif arm =="r":
            self.command_joint_pose('r', r, self.timeout, blocking=blocking)
