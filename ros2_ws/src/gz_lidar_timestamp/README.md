# gz_lidar_timestamp

Appends a **per-point relative timestamp** to a Gazebo `gpu_lidar`
`sensor_msgs/PointCloud2` so lidar odometry can **deskew**.

Gazebo's `gpu_lidar` emits `x,y,z,intensity,ring` but **no per-point time**, and
there is no SDF option to add one. Without it, LIO-SAM disables deskew ("system
will drift significantly!") and KISS-ICP runs deskew-off. This node intercepts
the cloud, computes each point's time within the sweep, appends the field, and
republishes. Background: PLAN §17.2 and `docs/kiss-icp-failure-modes.md` (#3).

## Run
```bash
colcon build --packages-select gz_lidar_timestamp
source install/setup.bash

ros2 launch gz_lidar_timestamp timestamp_injector.launch.py \
    input_topic:=/a200_0000/sensors/lidar3d_0/points \
    output_topic:=/a200_0000/sensors/lidar3d_0/points_with_time \
    scan_rate_hz:=10.0 profile:=velodyne
# or: ros2 run gz_lidar_timestamp timestamp_injector --ros-args --params-file config/params.yaml
```

Point your odometry node at the `output_topic`.

## Parameters
| param | default | meaning |
|---|---|---|
| `input_topic` / `output_topic` | `/cloud_in` / `/cloud_with_time` | gz cloud in, timestamped cloud out |
| `scan_rate_hz` | `10.0` | sweep rate → period `T = 1/rate` |
| `profile` | `velodyne` | `velodyne` → field `time` FLOAT32 sec; `ouster` → field `t` UINT32 ns. **Must match the consumer.** |
| `method` | `auto` | `column` (organized cloud, exact), `azimuth` (`atan2(y,x)`), or `auto` |
| `time_reference` | `start` | `t` in `[0,T)` from scan start, or `end` → `[-T,0)` |
| `clockwise` | `false` | lidar spin direction |
| `azimuth_start_rad` | `-pi` | reference azimuth for the `azimuth` method |

## Verify
```bash
ros2 topic echo --once <output_topic> --field fields    # lists the new 'time'/'t'
```
The added field's values should fall in `[0, T)` (e.g. `[0, 0.1)` at 10 Hz).

## Notes / limits
- Assumes little-endian point data (true for gz on x86).
- `column` (`t = col / width * T`, in `[0, T)`) is exact for a spinning lidar
  emitted in scan order; `azimuth` is the fallback for unorganized clouds and
  assumes a constant spin rate.
- Pass-through if the cloud already carries the target field (real Ouster/Velodyne).
