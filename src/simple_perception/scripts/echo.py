#!/usr/bin/env python
import pcl
import struct, ctypes
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from cardb.srv import SegmentScene, SegmentSceneRequest, DetectModels, DetectModelsRequest
from geometry_msgs.msg import Polygon,Point, PolygonStamped, Pose
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import threading
import ros_numpy
from tf2_ros import TransformStamped, TransformListener, Buffer
import tf2_sensor_msgs
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive

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
        self.pub = rospy.Publisher("~echo_image",\
                PointCloud2, queue_size=1)
        self.tablepub = rospy.Publisher("table",\
                PointCloud2, queue_size=1)
        self.nottablepub = rospy.Publisher("tablenot",\
                PointCloud2, queue_size=1)
        
        self.planepub = rospy.Publisher("mytable", \
                PolygonStamped, queue_size=1)
        self.pub_collision = rospy.Publisher("/collision_object", CollisionObject, queue_size=1, latch=True)

        
        self.detsrv = rospy.ServiceProxy('/detect_models_soda',\
                DetectModels)

        rospy.loginfo("[%s] Initialized." %(self.node_name))
        
    def latestTransform(self):
        rospy.sleep(.5)
        print "listening to transforms!"
        while not rospy.is_shutdown():
            self.r.sleep()
            try:
                self.trans = self.buff.lookup_transform('base_link', 'kinect2_ir_optical_frame', rospy.Time())
                self.T =  tf2_sensor_msgs.transform_to_kdl(self.trans) 

            except:
                print "lookup error"
            
            self.r.sleep()

    def cb(self,msg):
        #self.lastHeader = msg.header
        thread = threading.Thread(target=self.process,args=(msg,))
        thread.setDaemon(True)
        thread.start()


    def pclToMsg(self, pc):
        try:
            msg = pcl_to_ros(pc)
        except:
            ptfields = [('x', 0), ('y', 4), ('z', 8)]
            fields = [PointField(name=idx, offset=offset,\
                datatype=PointField.FLOAT32, count=1) for \
                (idx, offset) in ptfields]
            msg = pc2.create_cloud_xyz32(self.lastHeader, np.asarray(pc))


        
        return msg

    def getPlanePoints(self, cloud):
        seg = cloud.make_segmenter_normals(ksearch=50)
        seg.set_optimize_coefficients(True)
        seg.set_model_type(pcl.SACMODEL_NORMAL_PLANE)
        seg.set_method_type(pcl.SAC_RANSAC)
        seg.set_distance_threshold(0.05)
        indices, model = seg.segment()
        cloud_plane = cloud.extract(indices, negative=False)
        
        chull = cloud_plane.make_ConcaveHull()
        chull.set_Alpha(1.3)
        cloud_plane = chull.reconstruct()

        
        points = [Point(p[0], p[1], p[2]) for p in np.asarray(cloud_plane)]
        poly=Polygon()
        poly.points = points


        return indices, cloud_plane, poly

    def process(self, msg):
        if not self.thread_lock.acquire(False):
            return
        msg = tf2_sensor_msgs.do_transform_cloud(msg, self.trans)
        msg.header.frame_id = "base_link"
        self.lastHeader = msg.header
        self.pub.publish(msg)

        pc = ros_to_pcl(msg, skip_nans=False) 
        pc = XYZRGB_to_XYZ(pc)
  
    
        fil = pc.make_passthrough_filter()
        fil.set_filter_field_name("z")
        fil.set_filter_limits(0.001, 1000)
        cloud_filtered = fil.filter()

        polygons = []
        detreq = DetectModelsRequest()
        detreq.models = ["soda"]
        
        detreq.max_dists_xy = [3]
        detreq.max_dists_z = [3]

        for i in range(3):
            indices, cloud_plane, poly = self.getPlanePoints(cloud_filtered)
            cloud_filtered = cloud_filtered.extract(indices, negative=True)
            new_msg = self.pclToMsg(cloud_plane)
            self.tablepub.publish(new_msg)
            new_msg = self.pclToMsg(cloud_filtered)
            self.nottablepub.publish(new_msg)
            detreq.surface_polygons.append(poly)
            
            
            pts = np.asarray(cloud_plane)
            dx,dy,dz = np.max(pts,axis=0) - np.min(pts, axis=0)
            max_x, max_y, max_z = np.max(pts, axis=0)
            min_x, min_y, min_z = np.min(pts, axis=0)
            if dz > 1 or max_z < 0.3:
                # not a table-- is a wall
                # or table is the floor
                continue
            else:
                ps = PolygonStamped(self.lastHeader, poly)
                self.planepub.publish(ps)
                collision_object = CollisionObject()
                collision_object.id = "table"
                collision_object.header = self.lastHeader
                object_shape = SolidPrimitive()
                object_shape.type = SolidPrimitive.BOX
                object_shape.dimensions.append(dx)
                object_shape.dimensions.append(dy)
                object_shape.dimensions.append(max_z)
                collision_object.primitives.append(object_shape)
                shape_pose = Pose()
                shape_pose.position.x = min_x  + dx/2
                shape_pose.position.y = min_y  + dy/2
                shape_pose.position.z = max_z/2
                shape_pose.orientation.w = 1.0
                collision_object.primitive_poses.append(shape_pose)
                collision_object.operation = CollisionObject.ADD
                self.pub_collision.publish(collision_object)



                collision_object.primitives.append(object_shape)



        tmp_pose = Pose()
        tmp_pose.orientation.x = 1
        detreq.initial_poses = [tmp_pose]

        detreq.header = self.lastHeader
        detreq.timeout = rospy.Duration(5)
        detreq.initial_poses = [Pose()]

        try:
            result = self.detsrv.call(detreq)
            print "success!"
            print result
        except:
            print "failed"

        """
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

