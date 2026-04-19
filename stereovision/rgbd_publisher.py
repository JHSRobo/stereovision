import rclpy
from rclpy.node import Node 

import cv2 
from cv_bridge import CvBridge
import depthai as dai 
import numpy as np 

from sensor_msgs.msg import Image, CameraInfo
from core.msg import RGBDImage

class RGBDPublisher(Node):
    def __init__(self):
        super().__init__('rgbd_publisher')

        self.log = self.get_logger()

        self.bridge = CvBridge()

        self.rgbd_pub = self.create_publisher(RGBDImage, '/camera/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)

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

        self.pipeline.start()

        self.create_timer(1/30, self.publish_frame)

    def publish_frame(self):
        frame = self.rgbd_queue.tryGet()
        if frame is None:
            return 

        depth = frame.getDepthFrame().getCvFrame()
        color = frame.getRGBFrame().getCvFrame()

        rgb_msg = self.bridge.cv2_to_imgmsg(color, encoding="bgr8")
        depth_msg = self.bridge.cv2_to_imgmsg(depth, encoding="16UC1")

        rgbd_msg = RGBDImage()
        rgbd_msg.rgb = rgb_msg
        rgbd_msg.depth = depth_msg

        intrinsics = frame.getDepthFrame().getTransformation().getSourceIntrinsicMatrix()

        info_msg = CameraInfo()
        info_msg.k = [
            intrinsics[0][0], 0.0, intrinsics[0][2], 
            0.0, intrinsics[1][1], intrinsics[1][2], 
            0.0, 0.0, 1.0
        ]

        self.rgbd_pub.publish(rgbd_msg)
        self.info_pub.publish(info_msg)

def main(args=None):
    rclpy.init(args=args)

    node = RGBDPublisher()

    try: rclpy.spin(node)
    except Exception as e: 
        node.log.info(str(e))
        node.log.info("Shutting down")
        node.pipeline.stop() # release camera upon program ending

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
