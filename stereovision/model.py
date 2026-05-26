import rclpy
from rclpy.node import Node 

import cv2 
import numpy as np 
import math

from sensor_msgs.msg import Image, CameraInfo, PointCloud2

class ModelNode(Node):
    def __init__(self):
        super().__init__('measure')

        self.log = self.get_logger()

        self.depth = None
        self.pcl = None

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.create_subscription(PointCloud2, '/depth_camera/pcl', self.pcl_callback, 10)
        self.create_subscription(CameraInfo, '/depth_camera/camera_info', self.info_callback, 10)


    def pcl_callback(self, msg):
        self.pcl = msg
        # if not self.freeze:
        #     self.depth = self.bridge.imgmsg_to_cv2(msg.depth, desired_encoding="16UC1")

    def info_callback(self, msg):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]


    # (pixel x-coord, pixel y-coord, depth)
    def get_world_point(self, u, v, z):
        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy 
        return (x, y, z)

    def get_distance(self, point_a, point_b):
        x1, y1, z1 = point_a
        x2, y2, z2 = point_b
        return math.sqrt((x2-x1)**2 + (y2-y1)**2 + (z2-z1)**2)

def main(args=None):
    rclpy.init(args=args)

    model_node = ModelNode()

    rclpy.spin(model_node)

    model_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

