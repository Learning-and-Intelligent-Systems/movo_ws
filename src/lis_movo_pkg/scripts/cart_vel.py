#!/usr/bin/env python
import rospy
from movo_msgs.msg import  JacoCartesianVelocityCmd
from std_msgs.msg import Header

class CartVel:
    def __init__(self):
        self.pub = rospy.Publisher("/movo/right_arm/cartesian_vel_cmd", \
                JacoCartesianVelocityCmd, queue_size=1)
        self.rate = rospy.Rate(50)

    def sendCmd(self, (x,y,z), (roll,pitch,yaw), time):
        cmd = JacoCartesianVelocityCmd()
        cmd.header.stamp = rospy.Time.now()
        cmd.x= x
        cmd.y = y
        cmd.z = z
        cmd.theta_x = roll
        cmd.theta_y = pitch
        cmd.theta_z = yaw
        
        end_time = rospy.Time.now() + rospy.Duration(time)
        
        rospy.loginfo( "sending cartesian vel command: %s " % cmd)

        while not rospy.is_shutdown() and rospy.Time.now() < end_time:
            self.pub.publish(cmd)
            self.rate.sleep()
        
        rospy.loginfo( "Done!")

    def translate(self, x,y,z):
        self.sendCmd((x,y,z), (0,0,0), .1)




if __name__=="__main__":
    rospy.init_node("cartveltester")
    cv = CartVel()
    #rospy.spin()


"""
rostopic pub -r 50 /movo/right_arm/cartesian_vel_cmd movo_msgs/JacoCartesianVelocityCmd 
"header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: 'base_link'
x: 0.1
y: 0.0 
z: 0.0
theta_x: 0.0
theta_y: 0.0
theta_z: 0.0" 
"""
