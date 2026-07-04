import depthai as dai, time, sys
fps_req = float(sys.argv[1]) if len(sys.argv) > 1 else 120
p = dai.Pipeline()
m = p.create(dai.node.MonoCamera)
m.setBoardSocket(dai.CameraBoardSocket.CAM_B)
m.setResolution(dai.MonoCameraProperties.SensorResolution.THE_480_P)
m.setFps(fps_req)
xo = p.create(dai.node.XLinkOut); xo.setStreamName("m"); m.out.link(xo.input)
with dai.Device(p) as dev:
    q = dev.getOutputQueue("m", 8, False)
    q.get()  # warmup
    n = 0; t0 = time.monotonic()
    while time.monotonic() - t0 < 5.0:
        q.get(); n += 1
    dt = time.monotonic() - t0
    print(f"v2 USB={dev.getUsbSpeed()} req={fps_req:.0f} -> delivered {n/dt:.1f} fps ({n} frames / {dt:.2f}s)")
