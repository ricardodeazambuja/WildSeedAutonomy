import depthai as dai, time
pipeline = dai.Pipeline()
mono = pipeline.create(dai.node.MonoCamera)
mono.setBoardSocket(dai.CameraBoardSocket.CAM_B)
mono.setResolution(dai.MonoCameraProperties.SensorResolution.THE_480_P)
mono.setFps(10)
xout = pipeline.create(dai.node.XLinkOut)
xout.setStreamName("mono")
mono.out.link(xout.input)
with dai.Device(pipeline) as device:
    print("USB speed:", device.getUsbSpeed(), "| IMU:", device.getConnectedIMU())
    q = device.getOutputQueue("mono", maxSize=4, blocking=False)
    n = 0; t0 = time.monotonic()
    while time.monotonic() - t0 < 4.0:
        f = q.tryGet()
        if f is not None:
            n += 1
            if n == 1: print("first frame:", f.getWidth(), "x", f.getHeight())
        time.sleep(0.005)
    print("RESULT v2 mono CAM_B:", n, "frames received")
