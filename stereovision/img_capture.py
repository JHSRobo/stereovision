import rclpy
from rclpy.node import Node 
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import PointCloud2, Image
import cv2
from cv_bridge import CvBridge
import open3d as o3d
from open3d_ros2_helper import open3d_ros2_helper as orh
import numpy as np
import os
from datetime import datetime

class ImgCaptureNode(Node):
    def __init__(self, path):
        super().__init__('img_capture')

        self.log = self.get_logger()

        self.path = path

        self.frame = None
        self.img_write_num = 0

        self.bridge = CvBridge()
        self.vid_capture = cv2.VideoCapture(f"http://192.168.88.94:5001/stream")

        cv2.namedWindow("Camera Feed", cv2.WND_PROP_FULLSCREEN)
        cv2.setWindowProperty("Camera Feed", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_NORMAL)

        self.recording = False
        self.declare_parameter("Recording", self.recording)

        self.add_on_set_parameters_callback(self.update_parameters)

        self.create_timer(0.01, self.img_callback)
        self.create_timer(3.0, self.write_img)

    def update_parameters(self, params):
        for param in params:
            if param.name == "Recording":
                self.recording  = param.value

        return SetParametersResult(successful=True)

    def img_callback(self):
        frame = self.read_frame()
        if frame is not None:
            self.frame = frame
            cv2.imshow("Camera Feed", frame)
            cv2.waitKey(1)

    def read_frame(self):
        success, frame = self.vid_capture.read()
        if not success:
            return None
        else: return frame

    def write_img(self):
        if self.recording and self.frame is not None:
            write_path = f"{self.path}/{self.img_write_num}.png"
            self.log.info(f"Writing Image #{self.img_write_num+1} to {write_path}")
            cv2.imwrite(write_path, self.frame)
            self.img_write_num += 1

def main(args=None):
    rclpy.init(args=args)

    subdirectory = datetime.now().strftime("%Y-%m-%d-%H:%M:%S")
    path = f"/home/jhsrobo/corews/src/stereovision/data/{subdirectory}"
    os.makedirs(path)

    img_capture = ImgCaptureNode(path)
    rclpy.spin(img_capture)
    img_capture.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

