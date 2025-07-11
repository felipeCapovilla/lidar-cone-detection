import numpy as np
import std_msgs.msg
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.clock import Clock
from sensor_msgs.msg import PointCloud2


def npArray_to_pc2(points_array: np.ndarray,frame_id = "lidar_frame",timestamp = None):
    header = std_msgs.msg.Header()
    if timestamp:
        header.stamp = timestamp
    else:
        header.stamp = Clock().now().to_msg()
    header.frame_id = frame_id

    cloud_msg = pc2.create_cloud_xyz32(header,points_array)

    return cloud_msg

def pc2_to_npArray(pc2_msg:PointCloud2)->np.ndarray:
    
    points_list = list(pc2.read_points(pc2_msg,field_names=('x','y','z'),skip_nans=True))
    pontos = np.array([(p[0],p[1],p[2]) for p in points_list],dtype=np.float32)

    return pontos

