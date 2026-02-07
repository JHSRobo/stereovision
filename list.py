import depthai as dai

# Prints out relavent camera data to each sensor

with dai.Device() as device:
    features = device.getConnectedCameraFeatures()
    for feature in features:
        print(f"Socket:", feature.socket)
        print("\tSensor name:", feature.sensorName)
        print("\tNative resolution:", feature.width, "x", feature.height)
        print("\tSupported types:", feature.supportedTypes)
        print("\tHas autofocus:", bool(feature.hasAutofocus))
        print()

    print(device.getPlatform())
