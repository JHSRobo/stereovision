import depthai as dai

# Prints out relavent camera data to each sensor

with dai.Device() as device:
    features = device.getConnectedCameraFeatures()
    calib = device.readCalibration()

    info = [{}, {}, {}]
    for i, feature in enumerate(features):
        print(f"Socket:", feature.socket)
        print("\tSensor name:", feature.sensorName)
        print("\tNative resolution:", feature.width, "x", feature.height)
        print("\tSupported types:", feature.supportedTypes)
        print("\tHas autofocus:", bool(feature.hasAutofocus))

        if i == 0:
            intrinsics = calib.getCameraIntrinsics(
                dai.CameraBoardSocket.CAM_A,
                1280, 800
            )
        elif i == 1:
            intrinsics = calib.getCameraIntrinsics(
                dai.CameraBoardSocket.CAM_B,
                1280, 800
            )
        elif i == 2:
            intrinsics = calib.getCameraIntrinsics(
                dai.CameraBoardSocket.CAM_C,
                1280, 800
            )

        fx = intrinsics[0][0]
        fy = intrinsics[1][1]
        cx = intrinsics[0][2]
        cy = intrinsics[1][2]

        print("\tfx, fy:", fx, fy)
        print("\tcx, cy:", cx, cy)

        print()
