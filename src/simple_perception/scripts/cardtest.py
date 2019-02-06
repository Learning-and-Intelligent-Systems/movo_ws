#!/usr/bin/env python
import pcl
import struct, ctypes
import rospy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import threading
import ros_numpy
from cardb.srv import SegmentScene, SegmentSceneRequest, DetectModels, DetectModelsRequest
from geometry_msgs.msg import Point, Polygon, PolygonStamped
from chull_helper import CHull
from tf2_ros import TransformStamped, TransformListener, Buffer
import tf2_sensor_msgs


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
        self.buff = Buffer()
        self.listener = TransformListener(self.buff)
        self.r =rospy.Rate(10)
        thread = threading.Thread(target=self.latestTransform)
        thread.start()

        self.sub = rospy.Subscriber("/kinect2/sd/points",\
                PointCloud2, self.cb, queue_size=1)
        self.srv = rospy.ServiceProxy('/segment_scene',
                SegmentScene)
        self.detsrv = rospy.ServiceProxy('/detect_models_soda',\
                DetectModels)
        self.tablepub = rospy.Publisher("mytable", \
                PolygonStamped, queue_size=1)

        #self.pub = rospy.Publisher("~echo_image",\
        #        PointCloud2, queue_size=1)
        #self.tablepub = rospy.Publisher("table",\
        #        PointCloud2, queue_size=1)
        #self.nottablepub = rospy.Publisher("tablenot",\
        #        PointCloud2, queue_size=1)
    
        rospy.loginfo("[%s] Initialized." %(self.node_name))
        
    def latestTransform(self):
        print "listening to transforms!"
        while not rospy.is_shutdown():
            self.r.sleep()
            #trans = self.buff.lookup_transform('kinect2_ir_optical_frame', 'base_link', rospy.Time())
            try:
                self.trans = self.buff.lookup_transform('kinect2_ir_optical_frame', 'base_link', rospy.Time())
            except:
                print "lookup error"
            
            self.r.sleep()

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
        msg = tf2_sensor_msgs.do_transform_cloud(msg, self.trans)
        pc = ros_to_pcl(msg) 
        pc = XYZRGB_to_XYZ(pc)
        req = SegmentSceneRequest()
        req.dist_threshold = 0.03
        req.angle_threshold = 0.1
        req.plane_dist_threshold = 0.02
        req.min_plane_size =  .25
        req.max_floor_height= 1.0
        req.cluster_tolerance= 0.05
        req.min_cluster_size= 0.05
        req.max_cluster_size= 1


        pts  = np.asarray(pc).tolist()
        #import pdb; pdb.set_trace()
        for (x,y,z) in pts:
            if np.isnan(x): continue
            req.points.append( Point(x,y,z))

        result = self.srv.call(req)
        
        detreq = DetectModelsRequest()
        detreq.models = ["soda"]
        detreq.max_dists_xy = [1]
        detreq.max_dists_z = [1]

        for wall in result.walls:
            poly = Polygon()

            points = [(p.x, p.y, p.z) for p in wall.points]
            try:
                ch = CHull(points)
            except:
                continue
            max_x, max_y, max_z = ch.max_pts()
            if max_z > 5: continue

            ch_pts = [ Point(x,y,z) for (x,y,z) in ch.get_vertices()]

            poly.points = ch_pts
            detreq.surface_polygons.append(poly)
            ps = PolygonStamped(self.lastHeader, poly)
            self.tablepub.publish(ps)
        detreq.header = self.lastHeader
        try:
            result = self.detsrv.call(detreq)
            print "success!"
            print result
        except:
            print "failed"



        """ 
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
        """


        self.thread_lock.release()



if __name__=="__main__":
    rospy.init_node('Echo')
    e = Echo()
    rospy.spin()

