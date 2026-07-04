import depthai as dai
import numpy as np
import time, sys

def run(req_rate, secs=6.0):
    accel_ts, gyro_ts, batch_sizes = [], [], []
    with dai.Pipeline() as pipeline:
        imu = pipeline.create(dai.node.IMU)
        imu.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW], req_rate)
        imu.setBatchReportThreshold(1)
        imu.setMaxBatchReports(20)
        q = imu.out.createOutputQueue(maxSize=200, blocking=False)
        pipeline.start()
        t0 = time.monotonic()
        while time.monotonic() - t0 < secs:
            data = q.tryGet()
            if data is None:
                time.sleep(0.001); continue
            pkts = data.packets
            batch_sizes.append(len(pkts))
            for p in pkts:
                accel_ts.append(p.acceleroMeter.getTimestampDevice().total_seconds())
                gyro_ts.append(p.gyroscope.getTimestampDevice().total_seconds())
    return np.array(accel_ts), np.array(gyro_ts), np.array(batch_sizes)

def stats(name, ts):
    ts = np.sort(np.unique(ts))
    if len(ts) < 3:
        print(f"  {name}: too few samples ({len(ts)})"); return
    dt = np.diff(ts) * 1000.0  # ms
    dur = ts[-1] - ts[0]
    print(f"  {name}: n={len(ts)}  eff_rate={len(ts)/dur:6.1f} Hz  "
          f"dt(ms) mean={dt.mean():.3f} std={dt.std():.3f} "
          f"min={dt.min():.3f} max={dt.max():.3f}  jitter(max-min)={dt.max()-dt.min():.3f}")

for rate in (200, 400):
    print(f"\n=== requested {rate} Hz ===")
    a, g, b = run(rate)
    stats("accel", a)
    stats("gyro ", g)
    if len(b): print(f"  batch sizes: mean={b.mean():.2f} max={b.max()} (packets per queue get)")
    # accel vs gyro time alignment
    n = min(len(a), len(g))
    if n: 
        off = (np.sort(a)[:n] - np.sort(g)[:n]) * 1000.0
        print(f"  accel-vs-gyro ts offset(ms): mean={off.mean():.3f} std={off.std():.3f}")
