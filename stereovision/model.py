import rclpy
from rclpy.node import Node 
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import PointCloud2
import open3d as o3d
from open3d_ros2_helper import open3d_ros2_helper as orh
import numpy as np
import os
import time
from datetime import datetime

class ModelNode(Node):
    def __init__(self):
        super().__init__('model')

        self.log = self.get_logger()

        subdirectory = datetime.now().strftime("%Y-%m-%d-%H:%M:%S")
        self.path = f"/home/jhsrobo/corews/src/stereovision/models/{subdirectory}"
        self.num = 0

        os.makedirs(self.path)

        self.pcl = None

        self.declare_parameter('Save Point Cloud Button', False)
        self.add_on_set_parameters_callback(self.button_pressed)

        self.create_subscription(PointCloud2, '/depth_camera/point_cloud', self.pcl_callback, 10)

    def button_pressed(self, params):
        if self.pcl != None:
            self.log.info(f"Saving PCL to {self.path}/{self.num}.ply")
            self.write_pcl(self.pcl)

        return SetParametersResult(successful=True)

    def pcl_callback(self, msg):
        # Convert ROS pointcloud to Open3D format.
        o3d_cloud = orh.rospc_to_o3dpc(msg)

        # Scale ros point cloud units (m) to o3d point cloud units (mm).
        pts = np.asarray(o3d_cloud.points)
        pts *= 1000.0 
        o3d_cloud.points = o3d.utility.Vector3dVector(pts)

        self.pcl = o3d_cloud

    def write_pcl(self, o3d_cloud):
        o3d.io.write_point_cloud(f"{self.path}/{self.num}.ply", o3d_cloud)
        self.num += 1

def main(args=None):
    rclpy.init(args=args)

    model_node = ModelNode()

    rclpy.spin(model_node)

    model_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

