import depthai as dai
import numpy as np, time

cam_ts, imu_ts = [], []
with dai.Pipeline() as pipeline:
    cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)  # left mono OV7251
    camOut = cam.requestFullResolutionOutput()
    camQ = camOut.createOutputQueue(maxSize=10, blocking=False)
    imu = pipeline.create(dai.node.IMU)
    imu.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW], 200)
    imu.setBatchReportThreshold(1); imu.setMaxBatchReports(20)
    imuQ = imu.out.createOutputQueue(maxSize=200, blocking=False)
    pipeline.start()
    t0 = time.monotonic()
    while time.monotonic() - t0 < 4.0:
        f = camQ.tryGet()
        if f is not None:
            cam_ts.append(f.getTimestampDevice().total_seconds())
        d = imuQ.tryGet()
        if d is not None:
            for p in d.packets:
                imu_ts.append(p.acceleroMeter.getTimestampDevice().total_seconds())
        time.sleep(0.001)

cam_ts = np.array(sorted(cam_ts)); imu_ts = np.array(sorted(imu_ts))
if len(cam_ts) < 2 or len(imu_ts) < 2:
    print(f"insufficient data: cam={len(cam_ts)} imu={len(imu_ts)}"); raise SystemExit
print(f"camera frames: n={len(cam_ts)}  rate={len(cam_ts)/(cam_ts[-1]-cam_ts[0]):.1f} Hz")
print(f"  cam device-ts range:  {cam_ts[0]:.4f} .. {cam_ts[-1]:.4f} s")
print(f"imu samples:   n={len(imu_ts)}  rate={len(imu_ts)/(imu_ts[-1]-imu_ts[0]):.1f} Hz")
print(f"  imu device-ts range:  {imu_ts[0]:.4f} .. {imu_ts[-1]:.4f} s")
lo, hi = max(cam_ts[0], imu_ts[0]), min(cam_ts[-1], imu_ts[-1])
print(f"\nshared-clock check: overlapping window = {hi-lo:.2f}s of common timeline")
gaps = [np.min(np.abs(imu_ts - c))*1000 for c in cam_ts if lo <= c <= hi]
if gaps:
    print(f"per-frame nearest-IMU gap (ms): mean={np.mean(gaps):.3f} max={np.max(gaps):.3f} "
          f"(<=~2.5ms => same clock; IMU sample within half-period of every frame)")
