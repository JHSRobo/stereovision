#!/usr/bin/env python3

import cv2
import depthai as dai
import time

with dai.Pipeline() as pipeline:
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
    
    pipeline.start()
 
    while pipeline.isRunning():
        data = q_rgbd.get()
        rgb_frame = data.getRGBFrame().getCvFrame()
        depth_frame = data.getDepthFrame()

        cv2.imshow('rgb', rgb_frame)
        
        if cv2.waitKey(1) == ord('q'):
            break
