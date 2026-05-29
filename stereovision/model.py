import rclpy
from rclpy.node import Node 
from sensor_msgs.msg import PointCloud2, Image
import cv2
from cv_bridge import CvBridge
import open3d as o3d
from open3d_ros2_helper import open3d_ros2_helper as orh
import numpy as np
import os
from datetime import datetime

class ModelNode(Node):
    def __init__(self, path):
        super().__init__('model')

        self.log = self.get_logger()

        self.bridge = CvBridge()

        self.rgb = None
        self.pcl = None

        self.path = path
        self.rgb_num = 0
        self.pcl_num = 0

        self.window_name = "rgb"
        cv2.namedWindow(self.window_name)

        self.create_subscription(PointCloud2, '/depth_camera/point_cloud', self.pcl_callback, 10)
        self.create_subscription(Image, '/depth_camera/image', self.img_callback, 10)

        self.add_pcl_button = False
        self.declare_parameter('Save Point Cloud', self.add_pcl_button)

        self.create_timer(0.1, self.check_button_pressed)

    def check_button_pressed(self):
        param_value = self.get_parameter('Save Point Cloud').value
        self.add_pcl_button = param_value
        if self.add_pcl_button != param_value:
            if self.pcl is not None:
                self.write_pcl(self.pcl)

            self.add_pcl_button = param_value

    def img_callback(self, msg):
        self.rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv2.imshow(self.window_name, self.rgb)
        cv2.waitKey(1)

        cv2.imwrite(f"{self.path}/rgb/{self.rgb_num}.png", self.rgb)
        self.rgb_num += 1

    def pcl_callback(self, msg):
        # Convert ROS pointcloud to Open3D format.
        o3d_cloud = orh.rospc_to_o3dpc(msg)

        # Scale ros point cloud units (m) to o3d point cloud units (mm).
        pts = np.asarray(o3d_cloud.points)
        pts *= 1000.0 
        o3d_cloud.points = o3d.utility.Vector3dVector(pts)

        self.pcl = o3d_cloud

    def write_pcl(self, o3d_cloud):
        self.log.info(f"Saving PCL to {self.path}/pcl/{self.pcl_num}.ply")
        o3d.io.write_point_cloud(f"{self.path}/pcl/{self.pcl_num}.ply", o3d_cloud)
        self.orientations.append(self.orientation.copy())
        self.pcl_num += 1

def main(args=None):
    rclpy.init(args=args)

    subdirectory = datetime.now().strftime("%Y-%m-%d-%H:%M:%S")
    path = f"/home/jhsrobo/corews/src/stereovision/data/{subdirectory}"
    os.makedirs(f"{path}/rgb")
    os.makedirs(f"{path}/pcl")

    model_node = ModelNode(path)
    rclpy.spin(model_node)
    model_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

