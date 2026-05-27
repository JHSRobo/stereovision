import rclpy
from rclpy.node import Node 
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
import open3d as o3d
from sensor_msgs_py import point_cloud2 as pc2

import numpy as np
import struct
import time

class ModelNode(Node):
    def __init__(self):
        super().__init__('measure')

        self.log = self.get_logger()

        self.create_subscription(PointCloud2, '/depth_camera/point_cloud', self.pcl_callback, 10)

    def pcl_callback(self, msg):
        points = pc2.read_points_list(msg, field_names=["x", "y", "z", "rgb"], skip_nans=True)

        xyz = np.array([[p.x, p.y, p.z] for p in points], dtype=np.float64)
        xyz /= 100.0
        xyz[:, 0] -= xyz[:, 0].min()  # shift X to start at 0
        # Unpack RGB from packed float
        colors = []
        for p in points:
            rgb_int = struct.unpack('I', struct.pack('f', p.rgb))[0]
            r = ((rgb_int >> 16) & 0xFF) / 255.0
            g = ((rgb_int >> 8)  & 0xFF) / 255.0
            b = ( rgb_int        & 0xFF) / 255.0
            colors.append([r, g, b])
        colors = np.array(colors, dtype=np.float64)

        o3d_cloud = o3d.geometry.PointCloud()
        o3d_cloud.points = o3d.utility.Vector3dVector(xyz)
        o3d_cloud.colors = o3d.utility.Vector3dVector(colors)

        o3d_cloud.translate(-o3d_cloud.get_center())

        self.log.info(f"xyz range: x={xyz[:,0].min():.2f}-{xyz[:,0].max():.2f}, "
              f"y={xyz[:,1].min():.2f}-{xyz[:,1].max():.2f}, "
              f"z={xyz[:,2].min():.2f}-{xyz[:,2].max():.2f}")

        o3d.io.write_point_cloud("output.ply", o3d_cloud)

        self.log.info("Done")
        exit()

    # # (pixel x-coord, pixel y-coord, depth)
    # def get_world_point(self, u, v, z):
    #     x = (u - self.cx) * z / self.fx
    #     y = (v - self.cy) * z / self.fy 
    #     return (x, y, z)
    #
    # def get_distance(self, point_a, point_b):
    #     x1, y1, z1 = point_a
    #     x2, y2, z2 = point_b
    #     return math.sqrt((x2-x1)**2 + (y2-y1)**2 + (z2-z1)**2)

def main(args=None):
    rclpy.init(args=args)

    model_node = ModelNode()

    rclpy.spin(model_node)

    model_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

