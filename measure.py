#!/usr/bin/env python3

import cv2
import depthai as dai
import numpy as np

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


with dai.Pipeline() as pipeline:
    # Constants
    res = (1280, 800)
    img_type = dai.ImgFrame.Type.NV12
    fps = 30

    left_cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    right_cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
    left_out = left_cam.requestOutput(size=res, type=img_type, fps=fps)
    right_out = right_cam.requestOutput(size=res, type=img_type, fps=fps)

    stereo = pipeline.create(dai.node.StereoDepth).build(
            left=left_out, 
            right=right_out,
            presetMode=dai.node.StereoDepth.PresetMode.DEFAULT)

    rgbd = pipeline.create(dai.node.RGBD).build()

    center = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    center_out = center.requestOutput(size=res, type=dai.ImgFrame.Type.RGB888i)
    center_out.link(rgbd.inColor)

    center_out.link(stereo.inputAlignTo)
    stereo.depth.link(rgbd.inDepth)

    q_rgbd = rgbd.rgbd.createOutputQueue(maxSize=8, blocking=False) # queue 
    # qPcl = rgbd.pcl.createOutputQueue()
    
    pipeline.start()

    colorMap = cv2.applyColorMap(np.arange(256, dtype=np.uint8), cv2.COLORMAP_JET)
    colorMap[0] = [0, 0, 0]  # to make zero-depth pixels black

    MIN_DEPTH = 300
    MAX_DEPTH = 3000
 
    while pipeline.isRunning():
        data = q_rgbd.get()
        # point_cloud = qPcl.get()

        rgb = data.getRGBFrame().getCvFrame() # type: ignore
        depth = data.getDepthFrame().getCvFrame() # type: ignore

        print(depth[400, 640])

        depthClipped = np.clip(depth, MIN_DEPTH, MAX_DEPTH)
        depthNormalized = ((depthClipped - MIN_DEPTH) / (MAX_DEPTH - MIN_DEPTH) * 255).astype(np.uint8)

        colorizedDepth = cv2.applyColorMap(depthNormalized, cv2.COLORMAP_JET)
        cv2.imshow("depth", colorizedDepth)
        key = cv2.waitKey(1)
        if key == ord('q'):
            pipeline.stop()
            break
