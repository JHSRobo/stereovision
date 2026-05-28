import rclpy
from rclpy.node import Node 
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Vector3, Quaternion
import open3d as o3d
from open3d_ros2_helper import open3d_ros2_helper as orh
import numpy as np
import os
import time
from datetime import datetime
from scipy.spatial.transform import Rotation

class ModelNode(Node):
    def __init__(self, path):
        super().__init__('model')

        self.log = self.get_logger()

        self.path = path
        self.num = 0

        self.pcl = None

        self.pcls = []
        self.orientations = []

        self.orientation = np.array([0.0, 0.0, 0.0, 1.0])

        self.create_subscription(PointCloud2, '/depth_camera/point_cloud', self.pcl_callback, 10)
        self.create_subscription(Quaternion, '/depth_camera/orientation', self.orientation_callback, 10)

        self.add_pcl_button = False
        self.declare_parameter('Save Point Cloud', self.add_pcl_button)

        self.model_button = False
        self.declare_parameter('Create Model', self.model_button)

        self.create_timer(0.1, self.check_button_pressed)

    def check_button_pressed(self):
        param_value = self.get_parameter('Save Point Cloud').value
        self.add_pcl_button = param_value
        # if self.add_pcl_button != param_value:
        #     if self.pcl is not None:
        #         self.log.info(f"Saving PCL to {self.path}/{self.num}.ply")
        #         self.write_pcl(self.pcl)
        #     self.add_pcl_button = param_value
        #
        param_value = self.get_parameter('Create Model').value
        # self.model_button = param_value
        if self.model_button != param_value:
            if self.num == 0:
                self.log.info("Save at least one point cloud before creating model.")
            else:
                self.create_model()

    def pcl_callback(self, msg):
        # Convert ROS pointcloud to Open3D format.
        o3d_cloud = orh.rospc_to_o3dpc(msg)

        # Scale ros point cloud units (m) to o3d point cloud units (mm).
        pts = np.asarray(o3d_cloud.points)
        pts *= 1000.0 
        o3d_cloud.points = o3d.utility.Vector3dVector(pts)

        self.pcl = o3d_cloud

        if self.add_pcl_button:
            self.log.info(f"Saving PCL to {self.path}/{self.num}.ply")
            self.write_pcl(self.pcl)

    def orientation_callback(self, msg):
        self.orientation = np.array([msg.x, msg.y, msg.z, msg.w])

    def create_model(self):
        self.log.info(f"Combining {self.num} point clouds...")

        combined = self.combine_pcls()
        o3d.io.write_point_cloud(f"{self.path}/pcl/combined.ply", combined)

        # mesh = self.pcl_to_mesh(combined)
        # o3d.io.write_triangle_mesh(f"{self.path}/mesh/mesh.ply", mesh)
        self.log.info("Done!")
        raise SystemExit

    def combine_pcls(self):
        combined = o3d.io.read_point_cloud(f"{self.path}/pcl/0.ply")
        combined = combined.voxel_down_sample(voxel_size=2.0)
        combined.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=10.0, max_nn=30)
        )

        T_ref = self.pose_to_transform(self.orientations[0])

        for i in range(1, self.num):
            source = o3d.io.read_point_cloud(f"{self.path}/pcl/{i}.ply")
            source = source.voxel_down_sample(voxel_size=2.0)
            source.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=10.0, max_nn=30)
            )

            T_i = self.pose_to_transform(self.orientations[i])
            T_rel = np.linalg.inv(T_ref) @ T_i

            coarse = o3d.pipelines.registration.registration_icp(
                source, combined,
                max_correspondence_distance=50.0,
                init=T_rel,
                estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane()
            )
            result = o3d.pipelines.registration.registration_colored_icp(
                source, combined,
                max_correspondence_distance=50.0,
                init=coarse.transformation,
                criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50)
            )

            source.transform(result.transformation)
            combined += source
            combined = combined.voxel_down_sample(voxel_size=5.0)
            combined.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=10.0, max_nn=30)
            )
            self.log.info(f"Registered cloud {i}, fitness: {result.fitness:.3f}")

        return combined

    def pose_to_transform(self, orientation):
        T = np.eye(4)
        T[:3, :3] = Rotation.from_quat(orientation).as_matrix()
        return T

    # def pcl_to_mesh(self, pcl):
    #     self.log.info("Beginning Point Cloud to Mesh Conversion")
    #
    #     pcl, _ = pcl.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    #     pcl = pcl.voxel_down_sample(voxel_size=2.0)
    #
    #     pcl.estimate_normals(
    #         search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=10.0, max_nn=30)
    #     )
    #     pcl.orient_normals_consistent_tangent_plane(100)
    #
    #     # Poisson reconstruction
    #     mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
    #         pcl,
    #         depth=9,
    #         scale=1.1,
    #         linear_fit=False
    #     )
    #
    #     # Trim noisy outer faces
    #     densities = np.asarray(densities)
    #     vertices_to_remove = densities < np.quantile(densities, 0.15)
    #     mesh.remove_vertices_by_mask(vertices_to_remove)
    #
    #     # Clean up
    #     mesh.remove_degenerate_triangles()
    #     mesh.remove_duplicated_triangles()
    #     mesh.remove_duplicated_vertices()
    #
    #     return mesh

    def write_pcl(self, o3d_cloud):
        o3d.io.write_point_cloud(f"{self.path}/pcl/{self.num}.ply", o3d_cloud)
        self.orientations.append(self.orientation.copy())
        self.num += 1

def main(args=None):
    rclpy.init(args=args)

    subdirectory = datetime.now().strftime("%Y-%m-%d-%H:%M:%S")
    path = f"/home/jhsrobo/corews/src/stereovision/models/{subdirectory}"
    os.makedirs(f"{path}/pcl")
    os.makedirs(f"{path}/mesh")

    model_node = ModelNode(path)

    rclpy.spin(model_node)

    model_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

