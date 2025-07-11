import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import PointCloud2 
from visualization_msgs.msg import Marker, MarkerArray
from .auxiliar_tools import *
from . import MLESAC,ROI_filter,DBSCAN


class DataProcessing(Node):
    def __init__(self):
        super().__init__('dataProcessing')

        self.mlesacPoints = np.empty((0,3),dtype=np.float32)
        self.ROIpoints = np.empty((0,3),dtype=np.float32)
        self.subscription = self.create_subscription(PointCloud2,'artificialLIDAR',self.listener_callback,1)
        self.mlesacPublisher = self.create_publisher(PointCloud2,'MLESAC_points',1)
        self.ROIPublisher = self.create_publisher(PointCloud2,'ROI_points',1)
        self.MarkerPublisher = self.create_publisher(MarkerArray,'visualization_marker',1)
        
    def listener_callback(self,msg):
        points = pc2_to_npArray(msg)
        
        ROI_points = ROI_filter.apply_ROI(points,-2,2,0,15,-1,-0.2)
        self.ROIpoints = ROI_points
        self.publish_ROIpoints()

        no_floor = MLESAC.remove_floor(ROI_points)
        self.mlesacPoints = no_floor
        self.publish_MLESACpoints()

        cones_center = DBSCAN.DBSCAN_cluster(no_floor)
        self.publish_coneMarkers(cones_center)

        

    def publish_ROIpoints(self):
        cloud_msg = npArray_to_pc2(self.ROIpoints,frame_id='lidar_frame')   
        self.ROIPublisher.publish(cloud_msg)

    def publish_MLESACpoints(self):
        cloud_msg = npArray_to_pc2(self.mlesacPoints,frame_id='lidar_frame')   
        self.mlesacPublisher.publish(cloud_msg)

    def publish_coneMarkers(self,centro_cones):
        marker_array = MarkerArray()

        for i, (x,y,z) in enumerate (centro_cones):
            marker = Marker()
            marker.header.frame_id = 'lidar_frame'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'cones'
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            marker.pose.position.x = float(x)
            marker.pose.position.y = float(y)
            marker.pose.position.z = float(z)
            
            marker.pose.orientation.x =0.0
            marker.pose.orientation.y =0.0
            marker.pose.orientation.z =0.0
            marker.pose.orientation.w =1.0

            marker.scale.x=0.2
            marker.scale.y=0.2
            marker.scale.z=0.2

            marker.color.r=0.0
            marker.color.g=0.0
            marker.color.b=1.0
            marker.color.a =0.7

            marker.lifetime.sec = 1
            marker_array.markers.append(marker)

        self.MarkerPublisher.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = DataProcessing()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ =='__main__':
    main()