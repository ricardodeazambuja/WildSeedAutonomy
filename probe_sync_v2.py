import depthai as dai, numpy as np, time
pipeline = dai.Pipeline()
mono = pipeline.create(dai.node.MonoCamera)
mono.setBoardSocket(dai.CameraBoardSocket.CAM_B)   # left mono OV7251
mono.setResolution(dai.MonoCameraProperties.SensorResolution.THE_480_P)
mono.setFps(30)
xoutC = pipeline.create(dai.node.XLinkOut); xoutC.setStreamName("mono"); mono.out.link(xoutC.input)
imu = pipeline.create(dai.node.IMU)
imu.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW], 200)
imu.setBatchReportThreshold(1); imu.setMaxBatchReports(20)
xoutI = pipeline.create(dai.node.XLinkOut); xoutI.setStreamName("imu"); imu.out.link(xoutI.input)

with dai.Device(pipeline) as device:
    print("USB speed:", device.getUsbSpeed())
    qC = device.getOutputQueue("mono", 30, False)
    qI = device.getOutputQueue("imu", 200, False)
    cam_ts, imu_ts = [], []
    t0 = time.monotonic()
    while time.monotonic() - t0 < 5.0:
        f = qC.tryGet()
        if f is not None: cam_ts.append(f.getTimestampDevice().total_seconds())
        d = qI.tryGet()
        if d is not None:
            for p in d.packets: imu_ts.append(p.acceleroMeter.getTimestampDevice().total_seconds())
        time.sleep(0.001)

cam_ts = np.array(sorted(cam_ts)); imu_ts = np.array(sorted(imu_ts))
print(f"camera: n={len(cam_ts)} rate={len(cam_ts)/(cam_ts[-1]-cam_ts[0]):.1f} Hz  ts=[{cam_ts[0]:.4f},{cam_ts[-1]:.4f}]s")
print(f"imu:    n={len(imu_ts)} rate={len(imu_ts)/(imu_ts[-1]-imu_ts[0]):.1f} Hz  ts=[{imu_ts[0]:.4f},{imu_ts[-1]:.4f}]s")
lo, hi = max(cam_ts[0], imu_ts[0]), min(cam_ts[-1], imu_ts[-1])
print(f"\nSHARED-CLOCK: common timeline overlap = {hi-lo:.2f}s")
gaps = np.array([np.min(np.abs(imu_ts - c))*1000 for c in cam_ts if lo <= c <= hi])
print(f"per-frame nearest-IMU gap (ms): mean={gaps.mean():.3f} max={gaps.max():.3f}")
print("  -> if <= ~2.5ms (half the 200Hz IMU period), camera & IMU are on ONE clock")
