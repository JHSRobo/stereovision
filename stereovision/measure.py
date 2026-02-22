import rclpy
from rclpy.node import Node 

import cv2 
import depthai as dai 
import numpy as np 
import math

class MeasureNode(Node):
    def __init__(self):
        super().__init__('measure')

        self.log = self.get_logger()
        
        self.freeze = False
        self.points = []

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

        # Intrinsics of the Depth Camera (not the rgb or mono cameras)
        self.depth = None
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        # Init and connections for partss of the camera to create a depth stream
        self.pipeline = dai.Pipeline()
        self.rgb = self.pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
        self.mono_left = self.pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
        self.mono_right = self.pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
        self.stereo = self.pipeline.create(dai.node.StereoDepth)

        self.rgb_out = self.rgb.requestOutput(size=(1280, 800), type=dai.ImgFrame.Type.RGB888i)
        self.mono_left_out = self.mono_left.requestFullResolutionOutput()
        self.mono_right_out = self.mono_right.requestFullResolutionOutput()

        self.rgb_out.link(self.stereo.inputAlignTo)
        self.mono_left_out.link(self.stereo.left)
        self.mono_right_out.link(self.stereo.right)

        self.stereo.setRectification(True)
        self.stereo.setExtendedDisparity(True)
        self.stereo.setLeftRightCheck(True)

        self.rgbd = self.pipeline.create(dai.node.RGBD).build()
        self.stereo.depth.link(self.rgbd.inDepth)
        self.rgb_out.link(self.rgbd.inColor)

        self.rgbd_queue = self.rgbd.rgbd.createOutputQueue(maxSize=8, blocking=False)

        self.pipeline.start()

        self.create_timer(1/30, self.receive_frame)

    def receive_frame(self):
        if not self.freeze:
            frame = self.rgbd_queue.tryGet()
            if frame is None: return

            self.depth = frame.getDepthFrame().getCvFrame() #type:ignore

            intrinsics = self.depth.getTransformation().getSourceIntrinsicMatrix()
            self.fx = intrinsics[0][0]
            self.fy = intrinsics[1][1]
            self.cx = intrinsics[0][2]
            self.cy = intrinsics[1][2]

            clipped_depth = np.clip(self.depth, self.MIN_DEPTH, self.MAX_DEPTH)
            normalized_depth = ((clipped_depth - self.MIN_DEPTH) / (self.MAX_DEPTH - self.MIN_DEPTH) * 255).astype(np.uint8)
            colorized_depth = cv2.applyColorMap(normalized_depth, cv2.COLORMAP_JET)

            cv2.imshow(self.window_name, colorized_depth)

    def click_callback(self, event, x, y, flags, param):
        # Make sure the click callback onlhy runs after a depth frame is received
        if self.depth is None: return 

        if event == cv2.EVENT_LBUTTONDOWN:
            if not self.freeze:
                self.freeze = True
                self.log.info("Freeze")
            else:
                z = self.depth[y, x] / 1000

                pnt = self.get_world_point(x, y, z, self.fx, self.fy, self.cx, self.cy)
                x, y, z = pnt 

                if z != 0:
                    self.log.info(f"Added Point: {pnt}")
                    self.points.append(pnt)
                else:
                    self.log.info("Invalid")

                if len(self.points) == 2:
                    distance = self.get_distance(self.points[0], self.points[1])
                    self.log.info(str(distance))

                    self.points.clear()
                    self.freeze = False

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

    try: rclpy.spin(measure_node)
    except Exception: 
        measure_node.pipeline.stop() # release camera upon program ending
        cv2.destroyAllWindows()

    measure_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
