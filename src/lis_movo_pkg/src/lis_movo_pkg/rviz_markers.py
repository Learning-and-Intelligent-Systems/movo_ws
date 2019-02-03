#!/usr/bin/env python

"""
Copyright (c) 2011, Willow Garage, Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Willow Garage, Inc. nor the names of its
      contributors may be used to endorse or promote products derived from
      this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
"""

import rospy
import threading

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
from lis_movo_pkg.uber_controller import Uber

SQRT2INV = 0.707106781

class SimpleRvizController:
    def __init__(self):

        self.thread_lock = threading.Lock()
        self.uc = Uber()
        self.server = InteractiveMarkerServer("movo_markers", q_size=1)
        self.createTorsoControl()


        
    def createNewMarker(self, frame_id, name, \
            interaction_mode, orientation_mode, (w,x,y,z)):

        int_marker = InteractiveMarker()
        int_marker.header.frame_id = frame_id
        int_marker.name = name
        int_marker.description = "" 

        control = InteractiveMarkerControl()
        control.interaction_mode = interaction_mode
        control.orientation_mode = orientation_mode
        control.orientation.w = 1*SQRT2INV
        control.orientation.x = 0*SQRT2INV
        control.orientation.y = 1*SQRT2INV
        control.orientation.z = 0*SQRT2INV

        # add the control to the interactive marker
        int_marker.controls.append(control);

        # add the interactive marker to our collection &
        self.server.insert(int_marker, self.cbMarker)

        # 'commit' changes and send to all clients
        self.server.applyChanges()

    def createTorsoControl(self):
        frame_id = "linear_actuator_link"
        name = "torso"
        interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        orientation_mode = InteractiveMarkerControl.FIXED
        w = SQRT2INV
        x = 0
        y = SQRT2INV
        z = 0
        quat = (w,x,y,z)

        self.createNewMarker(frame_id, name, \
                interaction_mode, orientation_mode, quat)

        

    def cbMarker(self, feedback):
        pose = feedback.pose.position
        fb = (pose.x, pose.y, pose.z)
        thread = threading.Thread(target=self.processFeedback,args=(fb,feedback.marker_name))
        thread.setDaemon(True)
        thread.start()


        feedback.pose.position.x = 0
        feedback.pose.position.y = 0
        feedback.pose.position.z = 0
        self.server.applyChanges()


    def processFeedback(self, fb, marker_name):
        if not self.thread_lock.acquire(False):
            return
        
        if marker_name=="torso":

            print "marker is now at" , fb 
            torso = self.uc.get_torso_pose()
            x,y,z = fb

            thresh = 0.01
            inc = 0.1
            time = 1


            if abs(z) > thresh: 
                print "about to send a command!"
            
            if z > thresh:
                self.uc.command_torso(torso + inc, time=time, blocking=True)
                print "send command!"

            elif z < -thresh:
                self.uc.command_torso(torso - inc, time=time, blocking=True)
                print "send command!"
            
        self.thread_lock.release()

if __name__=="__main__":
    rospy.init_node("simple_marker")
    src = SimpleRvizController()
    rospy.spin()
    
