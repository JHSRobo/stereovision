#!/usr/bin/env python3

import cv2
import depthai as dai

with dai.Pipeline() as pipeline:
    res = (1280, 800)
    img_type = dai.ImgFrame.Type.NV12
    fps = 60

    left_cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    left_out = left_cam.requestOutput(size=res, type=img_type, fps=fps).createOutputQueue()

    right_cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
    right_out = right_cam.requestOutput(size=res, type=img_type, fps=fps).createOutputQueue()

    device = pipeline.getDefaultDevice()
    pipeline.start()

    while pipeline.isRunning():
        left_frame = left_out.get()
        right_frame = right_out.get()

        cv2.imshow('left', left_frame.getCvFrame())
        cv2.imshow('right', right_frame.getCvFrame())

        if cv2.waitKey(1) == ord('q'):
            break
