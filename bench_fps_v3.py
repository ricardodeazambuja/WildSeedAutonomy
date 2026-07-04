import depthai as dai, time, sys
fps_req = float(sys.argv[1]) if len(sys.argv) > 1 else 120
with dai.Pipeline() as p:
    cam = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    out = cam.requestOutput((640, 480), type=dai.ImgFrame.Type.GRAY8, fps=fps_req)
    q = out.createOutputQueue(maxSize=8, blocking=False)
    p.start()
    dev = p.getDefaultDevice()
    q.get()  # warmup
    n = 0; t0 = time.monotonic()
    while time.monotonic() - t0 < 5.0:
        q.get(); n += 1
    dt = time.monotonic() - t0
    usb = dev.getUsbSpeed() if dev else "?"
    print(f"v3 USB={usb} req={fps_req:.0f} -> delivered {n/dt:.1f} fps ({n} frames / {dt:.2f}s)")
