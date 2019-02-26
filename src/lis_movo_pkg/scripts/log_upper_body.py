#!/usr/bin/env python
#Author: Ariel Anders

from lis_movo_pkg.uber_controller import Uber
import rospy
from tuckarms import MovoUpperBody
from argparse import ArgumentParser

filename = "logged_poses.py"

class LogUpperBody():
    def __init__(self, filename):
        self.mub = MovoUpperBody()
        self.uc = Uber()
        self.poses = []
        self.filename = filename


    def getUB(self):
        pos = [self.uc.joint_positions[j] for j in self.mub.upper_body_joints]
        self.poses.append(pos)
        return pos

    def printPoses(self):
        pts = "poses=[\n"
        for i,pos in enumerate(self.poses):
            pts+= "[%s],\n"%(",".join(["%.3f" % s for s in pos]))
        pts+="]\n"
        return pts

    def savePoses(self):
        with open(self.filename, "wb") as f:
            f.write(self.printPoses())
    
    def loadPoses(self):
        if len(self.poses) > 0:
            q = raw_input("already have poses saved.  Overwrite poses? [n]o")
            if q == "n":
                return self.poses

        try:
            exec("from %s import poses" % self.filename.replace('.py',''))
        except:
            poses = []

        self.poses = poses
        return poses

    def moveThroughPoses(self):
        for i, pose in enumerate(self.poses):
            print "Pose %d/%d " % (i+1, len(self.poses))
            raw_input("hit any key to move")
            self.mub.move(pose)

    def startLogging(self):
        print "Starting to log poses!"
        q = None
        while q!='q' and not rospy.is_shutdown():
            q = raw_input("Hit any key to save pose. Type [q] to quit")
            if q != 'q':
                print self.getUB()
        print "done logging poses.  Saving to file %s " % self.filename
        self.savePoses()


            





if __name__=="__main__":
    rospy.init_node("getupperbody")
    lub = LogUpperBody(filename)
    
    parser = ArgumentParser(description="Log poses or move through logged poses")
    parser.add_argument("-l", "--log", \
            help="Save upper body poses to file", action='store_true')
    parser.add_argument("-m", "--move", \
            help="Move through upper body poses", action='store_true')
    args = parser.parse_args()
    
    poses = lub.loadPoses() 
    if len(poses) > 0:
        q = raw_input( "use previously stored poses? [y]es or no")
        if q == "y":
            lub.poses += poses
            print "using previously stored poses"

    if len(poses) == 0:
        print "no poses saved, must log poses before moving"
        do_log = True
    else:
        do_log = args.log


    if do_log:
        lub.startLogging()
            

    if args.move:
        lub.moveThroughPoses()


 
