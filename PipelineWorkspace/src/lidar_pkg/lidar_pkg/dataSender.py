import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from .auxiliar_tools import *
from visualization_msgs.msg import Marker, MarkerArray

class DataSender(Node):
    def __init__(self):
        super().__init__('data_sender')

        self.data_publisher = self.create_publisher(PointCloud2, 'artificialLIDAR', 1)

        pontos = np.load('/home/felipe-capovilla/Documents/E-Racing/LIDAR/PipelineWorkspace/src/lidar_pkg/lidar_pkg/conesPC2.npy') #Load the file with PC2.
        self.final_file = npArray_to_pc2(pontos)

        timer_period = 1.0  #1 Hz.
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info('DataSender node started and publishing point cloud.')

    def timer_callback(self):
        self.data_publisher.publish(self.final_file)
        self.get_logger().debug('Published point cloud message.')

def main(args=None):
    rclpy.init(args=args)
    node = DataSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
