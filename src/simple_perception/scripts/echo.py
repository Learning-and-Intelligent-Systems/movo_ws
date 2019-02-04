#!/usr/bin/env python
import pcl
import struct, ctypes
import rospy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import threading
import ros_numpy
from simple_perception.pcl_helper import ( \
        ros_to_pcl, 
        pcl_to_ros,
        XYZRGB_to_XYZ,
        XYZ_to_XYZRGB)



class Echo:
    def __init__(self):
        self.node_name = "Echo"
        self.thread_lock = threading.Lock()
        self.lastHeader = None

        self.sub = rospy.Subscriber("/kinect2/sd/points",\
                PointCloud2, self.cb, queue_size=1)
        self.pub = rospy.Publisher("~echo_image",\
                PointCloud2, queue_size=1)
        self.tablepub = rospy.Publisher("table",\
                PointCloud2, queue_size=1)
        self.nottablepub = rospy.Publisher("tablenot",\
                PointCloud2, queue_size=1)

        rospy.loginfo("[%s] Initialized." %(self.node_name))
        

    def cb(self,msg):
        self.lastHeader = msg.header
        thread = threading.Thread(target=self.process,args=(msg,))
        thread.setDaemon(True)
        thread.start()


    def pclToMsg(self, pc):
        msg = pcl_to_ros(pc)
        msg.header = self.lastHeader
        return msg



    def process(self, msg):
        if not self.thread_lock.acquire(False):
            return
        pc = ros_to_pcl(msg) 
        
        fil = pc.make_passthrough_filter()
        fil.set_filter_field_name("z")
        fil.set_filter_limits(.5, 1.5)
        cloud_filtered = fil.filter()
        seg = cloud_filtered.make_segmenter_normals(ksearch=50)
        seg.set_optimize_coefficients(True)
        seg.set_model_type(pcl.SACMODEL_NORMAL_PLANE)
        seg.set_normal_distance_weight(0.05)
        seg.set_method_type(pcl.SAC_RANSAC)
        seg.set_max_iterations(300)
        seg.set_distance_threshold(0.03)
        indices, model = seg.segment()

        print(model)

        cloud_plane = cloud_filtered.extract(indices, negative=False)
        new_msg = self.pclToMsg(cloud_plane)
        self.tablepub.publish(new_msg)


        cloud_cyl = cloud_filtered.extract(indices, negative=True)
        new_msg = self.pclToMsg(cloud_cyl)
        self.nottablepub.publish(new_msg)

        seg = cloud_cyl.make_segmenter_normals(ksearch=50)
        seg.set_optimize_coefficients(True)
        seg.set_model_type(pcl.SACMODEL_PLANE)
        seg.set_normal_distance_weight(0.1)
        seg.set_method_type(pcl.SAC_RANSAC)
        seg.set_max_iterations(10000)
        seg.set_distance_threshold(0.05)
        #seg.set_radius_limits(0, 0.5)
        indices, model = seg.segment()

        print(model)

        cloud_cylinder = cloud_cyl.extract(indices, negative=False)
        new_msg = self.pclToMsg(cloud_cylinder)
        self.pub.publish(new_msg)


        self.thread_lock.release()



if __name__=="__main__":
    rospy.init_node('Echo')
    e = Echo()
    rospy.spin()

