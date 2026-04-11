import rclpy
from rclpy.node import Node 

import cv2 
import numpy as np 
import math

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

class MeasureNode(Node):
    def __init__(self):
        super().__init__('measure')

        self.bridge = CvBridge()
        self.log = self.get_logger()

        self.depth = None
        self.rgb = None

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.freeze = False
        self.points = []

        self.create_subscription(Image, '/rgb/image_raw',self.rgb_callback, 10)
        self.create_subscription(Image, '/depth/image_raw',self.depth_callback, 10)
        self.create_subscription(CameraInfo, '/camera_info',self.info_callback, 10)

        self.window_name = "depth"
        cv2.namedWindow(self.window_name)
        cv2.setMouseCallback(self.window_name, self.click_callback)

        # Creating a color map to display close depths as red and far depths as blue
        self.color_map = cv2.applyColorMap(np.arange(256, dtype=np.uint8), cv2.COLORMAP_JET)
        self.color_map[0] = [0, 0, 0]  # to make zero-depth pixels black

        # For the colorized depth stream, limit the color-range based on these depths.
        self.MIN_DEPTH = 300 
        self.MAX_DEPTH = 3000

        self.window_name = "depth"

        cv2.namedWindow(self.window_name, cv2.WND_PROP_FULLSCREEN)
        cv2.setWindowProperty(self.window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(self.window_name, self.click_callback)

        self.create_timer(1/30, self.display)

    def rgb_callback(self, msg):
        self.rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def depth_callback(self, msg):
        self.depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="16UC1")

    def info_callback(self, msg):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def display(self):
        if self.depth is None:
            return

        clipped_depth = np.clip(self.depth, self.MIN_DEPTH, self.MAX_DEPTH)
        normalized_depth = ((clipped_depth - self.MIN_DEPTH) / (self.MAX_DEPTH - self.MIN_DEPTH) * 255).astype(np.uint8)
        colorized_depth = cv2.applyColorMap(normalized_depth, cv2.COLORMAP_JET)

        cv2.imshow(self.window_name, colorized_depth)
        cv2.waitKey(1)

    def click_callback(self, event, x, y, flags, param):
        if self.depth is None or self.fx is None:
            return

        if event == cv2.EVENT_LBUTTONDOWN:
            if not self.freeze:
                self.freeze = True
                return

            z = self.depth[y, x] / 1000.0
            point = self.get_world_point(x, y, z)

            if z != 0:
                self.points.append(point)

            if len(self.points) == 2:
                d = self.get_distance(self.points[0], self.points[1])
                self.log.info(f"Distance: {d}")

                self.points.clear()
                self.freeze= False

    # (x-coord, y-coord, x focal-length, y focal-length, principal pnt x-coord, principal pnt y-coord)
    def get_world_point(self, u, v, z, fx, fy, cx, cy):
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy 
        return (x, y, z)

    def get_distance(self, point_a, point_b):
        x1, y1, z1 = point_a
        x2, y2, z2 = point_b
        return math.sqrt((x2-x1)**2 + (y2-y1)**2 + (z2-z1)**2)

def main(args=None):
    rclpy.init(args=args)

    measure_node = MeasureNode()

    rclpy.spin(measure_node)

    measure_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
