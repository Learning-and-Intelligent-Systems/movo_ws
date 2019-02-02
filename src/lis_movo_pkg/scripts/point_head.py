#!/usr/bin/env python
# Author: Ariel Anders
# simple program to look at clicked point in rviz
import rospy
from control_msgs.msg import PointHeadAction, PointHeadActionGoal, PointHeadGoal
from actionlib import SimpleActionClient
from geometry_msgs.msg import PointStamped, Point
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus


class HeadPointer:
    def __init__(self):
        self.pub = rospy.Publisher(\
                "/movo/head_controller/point_head_action/goal",\
            PointHeadActionGoal, queue_size=1)
        self.sub = rospy.Subscriber("/clicked_point", PointStamped,\
                self.cb, queue_size=1)


    def cb(self, msg):
        rospy.loginfo("received point click: %s " % msg.point)
        self.lookAt(msg)

    def lookAt(self, point):
        rospy.loginfo("sending look command %s" % point.point)
        goal = PointHeadGoal()
        goal.target = point
        goal.pointing_axis.x = 1.0
        goal.pointing_frame = "kinect2_link"
        goal.min_duration.secs = 2.0
        goal.max_velocity = 1.0
        
        action_goal = PointHeadActionGoal()
        action_goal.goal = goal

        self.pub.publish(action_goal)

if __name__ == "__main__":
    rospy.init_node("headpointer")
    hp = HeadPointer()
    rospy.spin()

        

