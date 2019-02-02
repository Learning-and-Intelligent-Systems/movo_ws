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

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
from lis_movo_pkg.uber_controller import Uber

class SimpleRvizController:
    def __init__(self):

        self.server = InteractiveMarkerServer("simple_marker", q_size=1)
        self.uc = Uber()
        
        # create an interactive marker for our server
        self.int_marker = InteractiveMarker()
        self.int_marker.header.frame_id = "linear_actuator_link"
        self.int_marker.name = "my_marker"
        self.int_marker.description = "Simple 1-DOF Control"

        # create a grey box marker
        self.box_marker = Marker()
        self.box_marker.type = Marker.CUBE
        self.box_marker.scale.x = 0.25
        self.box_marker.scale.y = 0.25
        self.box_marker.scale.z = 0.25
        self.box_marker.color.r = 0.0
        self.box_marker.color.g = 0.5
        self.box_marker.color.b = 0.5
        self.box_marker.color.a = 1.0

        # create a non-interactive control which contains the box
        self.box_control = InteractiveMarkerControl()
        self.box_control.always_visible = True
        self.box_control.markers.append( self.box_marker )

        # add the control to the interactive marker
        self.int_marker.controls.append( self.box_control )

        # create a control which will move the box
        # this control does not contain any markers,
        # which will cause RViz to insert two arrows
        self.rotate_control = InteractiveMarkerControl()
        self.rotate_control.name = "move_x"
        self.rotate_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        self.rotate_control.orientation_mode = InteractiveMarkerControl.FIXED
        SQRT2INV = 0.707106781;
        self.rotate_control.orientation.w = 1*SQRT2INV
        self.rotate_control.orientation.x = 0*SQRT2INV
        self.rotate_control.orientation.y = 1*SQRT2INV
        self.rotate_control.orientation.z = 0*SQRT2INV



        # add the control to the interactive marker
        self.int_marker.controls.append(self.rotate_control);

        # add the interactive marker to our collection &
        # tell the server to call processFeedback() when feedback arrives for it
        self.server.insert(self.int_marker, self.processFeedback)

        # 'commit' changes and send to all clients
        self.server.applyChanges()


    def processFeedback(self, feedback):
        p = feedback.pose.position
        print feedback.marker_name + " is now at " + str(p.x) + ", " + str(p.y) + ", " + str(p.z)

        torso = self.uc.get_torso_pose()
        if p.z > 0.1:
            self.uc.command_torso(torso+0.025, time=.2, blocking=True)

        elif p.z < -0.1:
            self.uc.command_torso(torso -0.025, time=.2, blocking=True)
            

        feedback.pose.position.x = 0
        feedback.pose.position.y = 0
        feedback.pose.position.z = 0


        self.server.applyChanges()

if __name__=="__main__":
    rospy.init_node("simple_marker")
    src = SimpleRvizController()
    rospy.spin()
    
