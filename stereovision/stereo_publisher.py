import rclpy
from rclpy.node import Node 
import cv2 
import numpy as np
from cv_bridge import CvBridge
import depthai as dai 
from sensor_msgs.msg import PointCloud2

class StereoPublisher(Node):
    def __init__(self):
        super().__init__('stereo_publisher')

        self.log = self.get_logger()

        self.bridge = CvBridge()

        self.pcl_pub = self.create_publisher(PointCloud2, '/depth_camera/point_cloud', 10)

        # Init and connections for parts of the camera to create a depth stream
        self.pipeline = dai.Pipeline()

        self.rgb = self.pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
        self.mono_left = self.pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
        self.mono_right = self.pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
        self.stereo = self.pipeline.create(dai.node.StereoDepth)

        self.rgb_out = self.rgb.requestOutput(size=(1280, 800), type=dai.ImgFrame.Type.RGB888i, enableUndistortion=True)
        self.mono_left_out = self.mono_left.requestFullResolutionOutput()
        self.mono_right_out = self.mono_right.requestFullResolutionOutput()

        self.rgb_out.link(self.stereo.inputAlignTo)
        self.mono_left_out.link(self.stereo.left)
        self.mono_right_out.link(self.stereo.right)

        self.stereo.setRectification(True)
        self.stereo.setExtendedDisparity(False)
        self.stereo.setLeftRightCheck(True)
        self.stereo.setSubpixel(True)

        self.rgbd = self.pipeline.create(dai.node.RGBD).build()
        self.stereo.depth.link(self.rgbd.inDepth)
        self.rgb_out.link(self.rgbd.inColor)

        self.rgbd_queue = self.rgbd.rgbd.createOutputQueue(maxSize=8, blocking=False)
        self.pcl_queue = self.rgbd.pcl.createOutputQueue(maxSize=8, blocking=False)

        self.pipeline.start()

        self.create_timer(0.1, self.publish_cam)

    def publish_cam(self):
        pcl = self.pcl_queue.tryGet()

        if pcl is not None:
            points, colors = pcl.getPointsRGB()
            msg = self.create_pcl_msg(points, colors)
            self.pcl_pub.publish(msg)

    def create_pcl_msg(self, points, colors):
        # Filter out invalid points
        dist = np.linalg.norm(points, axis=1)
        valid = (
            np.isfinite(points).all(axis=1) &
            (dist > 100) &      # 10 cm
            (dist < 10000)       # 10 meters
            )
        points = points[valid]
        colors = colors[valid]

        length = len(points)
        # Placing the rgb values into a single byte
        r = colors[:, 0].astype(np.uint32)
        g = colors[:, 1].astype(np.uint32)
        b = colors[:, 2].astype(np.uint32)

        rgb_packed = (r << 16) | (g << 8) | b
        # Converting rgb value to a float to fit ros convention
        rgb_float = rgb_packed.view(np.float32)

        cloud = np.zeros(length, dtype=[
                ('x', np.float32),
                ('y', np.float32),
                ('z', np.float32),
                ('rgb', np.float32)
            ])

        # Remap axes from depthai convention to ros convention, change units from mm to m
        cloud['x'] = points[:, 2] / 1000.0# Z -> X
        cloud['y'] = -points[:, 0] / 1000.0 # X -> -Y
        cloud['z'] = -points[:, 1] / 1000.0 #Y -> -Z
        cloud['rgb'] = rgb_float

        msg = PointCloud2()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        msg.height = 1
        msg.width = length
        msg.fields = [
            PointField(name='x', offset = 0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset = 4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset = 8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset = 12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = 16 * length
        msg.data = cloud.tobytes()
        msg.is_dense = True
        return msg

def main(args=None):
    rclpy.init(args=args)

    node = StereoPublisher()

    try: rclpy.spin(node)
    except Exception as e: 
        node.log.info(str(e))
        node.log.info("Shutting down")
        node.pipeline.stop() # release camera upon program ending

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
