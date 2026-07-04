import depthai as dai
import sys, time

socket = {"A": dai.CameraBoardSocket.CAM_A, "B": dai.CameraBoardSocket.CAM_B,
          "C": dai.CameraBoardSocket.CAM_C}[sys.argv[1]]
size = (int(sys.argv[2]), int(sys.argv[3])) if len(sys.argv) > 3 else None
n = 0
with dai.Pipeline() as pipeline:
    cam = pipeline.create(dai.node.Camera).build(socket)
    if size:
        out = cam.requestOutput(size, type=dai.ImgFrame.Type.NV12, fps=10)
        print(f"requestOutput({size}, NV12, 10fps) on CAM_{sys.argv[1]}")
    else:
        out = cam.requestFullResolutionOutput()
        print(f"requestFullResolutionOutput() on CAM_{sys.argv[1]}")
    q = out.createOutputQueue(maxSize=4, blocking=False)
    pipeline.start()
    t0 = time.monotonic()
    while time.monotonic() - t0 < 4.0:
        f = q.tryGet()
        if f is not None:
            n += 1
            if n == 1:
                print(f"  first frame: {f.getWidth()}x{f.getHeight()} type={f.getType()}")
        time.sleep(0.005)
print(f"RESULT CAM_{sys.argv[1]}: received {n} frames")
