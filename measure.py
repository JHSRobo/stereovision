import cv2
import depthai as dai
import numpy as np
import math

global points
points = []

global freeze
freeze = False

def click_callback(event, x, y, flags, param):
    global freeze


    if event == cv2.EVENT_LBUTTONDOWN:
        if not freeze:
            freeze = True
            print("Freeze")
        else:
            z = depth[y, x] / 1000

            pnt = get_world_point(x, y, z, fx, fy, cx, cy)
            x, y, z = pnt 

            global points

            if z != 0:
                print(pnt)
                points.append(pnt)
            else:
                print("Invalid")

            if len(points) == 2:
                dist()
                points = []
                freeze = False
                print()

# (x-coord, y-coord, x focal-length, y focal-length, principal pnt x-coord, principal pnt y-coord)
def get_world_point(u, v, z, fx, fy, cx, cy):
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy 

    return (x, y, z)

def dist():
    print(points[0])
    print(points[1])
    x1, y1, z1 = points[0]
    x2, y2, z2 = points[1]

    print(math.sqrt((x2-x1)**2 + (y2-y1)**2 + (z2-z1)**2))

with dai.Device() as device:
    calib = device.readCalibration()
    intrinsics = calib.getCameraIntrinsics(
        dai.CameraBoardSocket.CAM_A,
        1280, 800
    )

    fx = intrinsics[0][0]
    fy = intrinsics[1][1]
    cx = intrinsics[0][2]
    cy = intrinsics[1][2]


pipeline = dai.Pipeline()
rgb = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
monoLeft = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
monoRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
stereo = pipeline.create(dai.node.StereoDepth)

# Linking
rgbOut = rgb.requestOutput(size=(1280, 800), type=dai.ImgFrame.Type.RGB888i)
monoLeftOut = monoLeft.requestFullResolutionOutput()
monoRightOut = monoRight.requestFullResolutionOutput()

rgbOut.link(stereo.inputAlignTo)
monoLeftOut.link(stereo.left)
monoRightOut.link(stereo.right)

stereo.setRectification(True)
stereo.setExtendedDisparity(True)
stereo.setLeftRightCheck(True)

rgbd = pipeline.create(dai.node.RGBD).build()
stereo.depth.link(rgbd.inDepth)
rgbOut.link(rgbd.inColor)

rgbdQueue = rgbd.rgbd.createOutputQueue(maxSize=8, blocking=False)

colorMap = cv2.applyColorMap(np.arange(256, dtype=np.uint8), cv2.COLORMAP_JET)
colorMap[0] = [0, 0, 0]  # to make zero-depth pixels black

# 800 x 1280

# 0.3 to 3 m range for color
MIN_DEPTH = 300
MAX_DEPTH = 3000

window_name = "rgbd"
    
cv2.namedWindow(window_name, cv2.WND_PROP_FULLSCREEN)
cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_NORMAL)
cv2.setMouseCallback(window_name, click_callback)

checked_intrinsics = False

with pipeline:
    pipeline.start()
    maxDepth = 1

    while pipeline.isRunning():
        if not freeze:
            frame = rgbdQueue.get()
            depth = frame.getDepthFrame().getCvFrame()

            if not checked_intrinsics:
                checked_intrinsics = True

                intrinsics = frame.getDepthFrame().getTransformation().getSourceIntrinsicMatrix()
                fx = intrinsics[0][0]
                fy = intrinsics[1][1]
                cx = intrinsics[0][2]
                cy = intrinsics[1][2]

            print(depth.size)

            depthClipped = np.clip(depth, MIN_DEPTH, MAX_DEPTH)
            depthNormalized = ((depthClipped - MIN_DEPTH) / (MAX_DEPTH - MIN_DEPTH) * 255).astype(np.uint8)

            colorizedDepth = cv2.applyColorMap(depthNormalized, cv2.COLORMAP_JET)
            cv2.imshow(window_name, colorizedDepth)
        key = cv2.waitKey(1)
        if key == ord('q'):
            pipeline.stop()
            break
