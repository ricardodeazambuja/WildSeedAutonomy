import depthai as dai
import numpy as np

with dai.Device() as device:
    print("Device:", device.getDeviceName(), "| IMU:", device.getConnectedIMU(),
          "| IMU FW:", device.getIMUFirmwareVersion())
    calib = device.readCalibration()
    sockets = {"CAM_A (RGB/color)": dai.CameraBoardSocket.CAM_A,
               "CAM_B (left mono)": dai.CameraBoardSocket.CAM_B,
               "CAM_C (right mono)": dai.CameraBoardSocket.CAM_C}
    np.set_printoptions(precision=4, suppress=True)
    for name, sock in sockets.items():
        print(f"\n=== IMU -> {name} extrinsics (4x4) ===")
        for spec in (False, True):
            try:
                M = np.array(calib.getImuToCameraExtrinsics(sock, spec))
                t_mm = M[:3, 3]
                print(f"  useSpecTranslation={spec}: translation(cm)={t_mm}")
                print("   R=\n", M[:3,:3])
            except Exception as e:
                print(f"  useSpecTranslation={spec}: ERROR {e}")
