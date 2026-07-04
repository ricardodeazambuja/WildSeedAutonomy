# Deep dive: how the 2D costmap is generated & updated

Companion to PLAN §18 (navigation). §18 stays at the architecture level; this is
the mechanism. Scope: Nav2 `costmap_2d` fed by the Husky's **Ouster OS1** 3D lidar
+ **OAK-D** depth, on the off-road `pipeline` world.

## 1. What a costmap cell holds
A costmap is an occupancy grid; each cell is one byte of *cost*:

| value | meaning |
|---|---|
| `0` | free space |
| `1`–`252` | inflation gradient (closer to an obstacle = higher) |
| `253` | inscribed — robot footprint would touch an obstacle |
| `254` | lethal — an obstacle is here |
| `255` | unknown / no information |

Planners treat `253`/`254` as impassable; the `1`–`252` gradient is what makes
paths *prefer* clearance rather than merely avoid collisions.

## 2. Two costmaps, two jobs
Nav2 runs **two** independent instances:

| | Global costmap | Local costmap |
|---|---|---|
| consumer | global **planner** (route to goal) | local **controller** (MPPI) |
| extent | whole known map | rolling window, e.g. 5×5 m around the robot |
| frame | `map` | `odom` |
| update freq | ~1 Hz | ~5–10 Hz (`update_frequency`) |
| typical layers | static + inflation (+ obstacle) | obstacle/voxel + inflation |

The local one is `rolling_window: true` — it re-centers on `base_link` every
cycle, so it's always a fresh patch of the world right around the robot.

## 3. Layered composition (the update cycle)
A costmap is a stack of plugin layers merged into one master grid each cycle:

```
 InflationLayer    grows lethal cells outward (exp. decay) → clearance gradient
 Voxel/ObstacleLayer   live sensor obstacles (mark + clear)
 StaticLayer       prebuilt / SLAM map (global only)
 ───────────────
 = master costmap   (planner / controller read this)
```

Each update cycle runs two passes over the layers:
1. **`updateBounds`** — every layer reports the bounding box it changed this tick
   (keeps work local; the whole grid isn't recomputed).
2. **`updateCosts`** — within that union box, layers write their costs into the
   master grid bottom-to-top, so higher layers (inflation) see lower layers' marks.

Layers in play here:
- **StaticLayer** — the `nav_msgs/OccupancyGrid` from `slam_toolbox` (full-auto)
  or a saved map. Backbone of the *global* costmap.
- **Obstacle / Voxel layer** — the live part (next section).
- **InflationLayer** — senses nothing; blurs lethal cells by `inflation_radius`
  with `cost_scaling_factor`, producing the clearance gradient MPPI rides.

## 4. The live layer: mark + clear by raytracing
For each configured **observation source**, every message does two operations:
- **Mark** — write lethal cost where a beam *hit* something.
- **Clear** — raytrace from the sensor origin to that hit and set intervening
  cells **free**. Clearing is what lets stale/false obstacles disappear.

```yaml
local_costmap:
  plugins: ["voxel_layer", "inflation_layer"]
  voxel_layer:
    plugin: "nav2_costmap_2d::VoxelLayer"
    observation_sources: ouster oakd
    ouster:
      topic: /a200_0000/sensors/lidar3d_0/points
      data_type: PointCloud2
      marking: true
      clearing: true
      obstacle_max_range: 25.0     # mark hits within this range
      raytrace_max_range: 30.0     # clear along rays out to this range
      min_obstacle_height: 0.10    # ← the off-road trap (§6)
      max_obstacle_height: 2.0
    oakd:
      topic: /a200_0000/sensors/camera_0/points
      data_type: PointCloud2
      marking: true
      clearing: true
      obstacle_max_range: 8.0      # depth is short-range but catches low/thin obstacles
```

The **VoxelLayer** raytraces in **3D voxels** first (so a 2 m overhang and a
0.2 m rock are distinguished), then projects occupied voxels **down** to the 2D
cell. Buffering knobs that matter: `observation_persistence` (how long a hit
lingers before it must be re-seen), `expected_update_rate` (flags a dead sensor),
`transform_tolerance` (TF slack).

> **For a 3D lidar, prefer STVL** — the Spatio-Temporal Voxel Layer
> (`spatio_temporal_voxel_layer`, separate package; verify on Jazzy). It stores a
> sparse 3D structure with **time-based decay** instead of per-tick raytrace
> clearing, which is markedly cheaper and cleaner for dense Ouster clouds. Drop-in
> replacement for VoxelLayer. The default VoxelLayer is the apt-only fallback.

## 5. Getting a 3D cloud into a 2D grid — two routes
1. **PointCloud2 → VoxelLayer/STVL directly** (above) — keeps height until the
   final flatten; best fidelity.
2. **`pointcloud_to_laserscan`** — slice a height band out of the cloud into one
   `LaserScan` ring, feed the lighter ObstacleLayer. Cheaper, discards vertical
   structure. Useful when CPU-bound.
   ```yaml
   # pointcloud_to_laserscan params
   min_height: 0.1
   max_height: 1.0
   range_max: 25.0
   ```

The **OAK-D depth** is a *complement*, not a substitute: short range, but it sees
low/thin obstacles directly ahead that a roof-mounted lidar's lowest ring skims
over. Add it as a second observation source.

## 6. The off-road problem (why §18 calls traversability a stretch)
`min_obstacle_height: 0.10` is the whole catch. On flat ground it rejects the
floor cleanly. But when the Husky **pitches on a slope** or terrain rolls, the
ground rises into the height band → the costmap marks **the hill you want to
climb as lethal**, and the planner refuses. Worse, **negative obstacles**
(ditches, drop-offs) reflect *no* return, so they read as free space — a real
hazard the 2D grid can't represent.

Mitigations, increasing effort:
1. **Height-band tuning** — works on `pipeline`'s gentle sections; brittle on real slopes.
2. **Ground-plane segmentation before the costmap** — RANSAC plane fit or
   grid-cell slope test removes ground points so only *non-ground* marks. Standard
   for mild off-road.
3. **Gravity-aligned height band** — transform the cloud into an
   **IMU-gravity-aligned frame** (we have a good Microstrain IMU, §8) before height
   filtering, so the "ground band" tilts with the robot and keeps rejecting ground
   on slopes. Cheap, high-leverage, on-thesis with the fusion spine.
4. **A real traversability layer** — score slope/roughness/step into a `gridmap`
   instead of binary occupancy (the "learned traversability → MPPI" frontier, §1).
   The stretch chapter, not the first pass.

## 7. Frames, TF & the fusion-spine payoff
Each costmap has a `global_frame` (`map` global / `odom` local) and
`robot_base_frame` (`base_link`). Every incoming cloud is transformed into the
costmap frame via **TF**, using the message timestamp (so `use_sim_time:=true`
everywhere — PLAN §17.2). `transform_tolerance` covers small TF lag.

Because the **local** costmap lives in **`odom`**, it inherits the dual-EKF
guarantee (§11): during a **GPS dropout**, `odom→base_link` stays smooth, so the
local costmap and the obstacles in it **don't jump** — the robot keeps avoiding
correctly mid-traverse. That is exactly the N3 money chart, and it's the concrete
reason the estimation spine is built before the planning layer.

## 8. How the controller consumes it
MPPI samples many short velocity rollouts each tick and scores them with
**critics**; the obstacle/cost critics penalize trajectories that pass through
high-cost cells of the **local** costmap, so the inflation gradient (§3) steers
the robot toward clearance, not just away from collisions. The **global** planner
separately searches the global costmap for the route to the goal. Same grid
machinery, two consumers.
