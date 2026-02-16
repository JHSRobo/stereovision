import cv2
import depthai as dai
import numpy as np

pipeline = dai.Pipeline()
rgb = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
monoLeft = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
monoRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
stereo = pipeline.create(dai.node.StereoDepth)

# Linking
rgbOut = rgb.requestOutput(size=(1280, 720), type=dai.ImgFrame.Type.RGB888i)
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

with pipeline:
    pipeline.start()
    maxDepth = 1

    while pipeline.isRunning():
        frame = rgbdQueue.get()
        color = frame.getRGBFrame()
        depth = frame.getDepthFrame().getCvFrame()
        print(depth.shape)
        print(len(depth))
        print(len(depth[0]))

        depthClipped = np.clip(depth, MIN_DEPTH, MAX_DEPTH)
        depthNormalized = ((depthClipped - MIN_DEPTH) / (MAX_DEPTH - MIN_DEPTH) * 255).astype(np.uint8)

        colorizedDepth = cv2.applyColorMap(depthNormalized, cv2.COLORMAP_JET)
        cv2.imshow("depth", colorizedDepth)
        key = cv2.waitKey(1)
        if key == ord('q'):
            pipeline.stop()
            break
