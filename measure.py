#!/usr/bin/env python3

import cv2
import depthai as dai

with dai.Pipeline() as pipeline:
    left_cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    left_out = left_cam.requestOutput((800, 600), dai.ImgFrame.Type.NV12, dai.ImgResizeMode.CROP, 20).createOutputQueue()

    right_cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
    right_out = right_cam.requestOutput((800, 600), dai.ImgFrame.Type.NV12, dai.ImgResizeMode.CROP, 20).createOutputQueue()

    device = pipeline.getDefaultDevice()
    pipeline.start()

    while pipeline.isRunning():
        left_frame = left_out.get()
        right_frame = right_out.get()

        cv2.imshow('left', left_frame.getCvFrame())
        cv2.imshow('right', right_frame.getCvFrame())

        if cv2.waitKey(1) == ord('q'):
            break
