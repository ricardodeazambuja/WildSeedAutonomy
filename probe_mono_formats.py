import depthai as dai, time, sys

# Try several output formats for the mono OV7251 (CAM_B) on depthai v3,
# to see if a mono-native format avoids the "Invalid config steps" crash.
types = []
for tname in ("GRAY8", "RAW8", "GRAY16", "RAW10", "NV12", "YUV420p"):
    t = getattr(dai.ImgFrame.Type, tname, None)
    if t is not None:
        types.append((tname, t))

tname, ttype = types[int(sys.argv[1])] if len(sys.argv) > 1 else types[0]
print(f"Trying CAM_B (OV7251 mono) requestOutput((640,480), {tname}, 10fps)")
n = 0
try:
    with dai.Pipeline() as pipeline:
        cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
        out = cam.requestOutput((640, 480), type=ttype, fps=10)
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
    print(f"RESULT {tname}: {n} frames  -> {'WORKS' if n>0 else 'no frames'}")
except Exception as e:
    print(f"RESULT {tname}: EXCEPTION {type(e).__name__}: {e}")
