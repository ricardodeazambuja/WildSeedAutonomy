#!/usr/bin/env python3
"""Headless Clearpath Husky sim — robust, event-driven (no fixed sleeps).

Why this exists: clearpath_gz's simulation.launch.py starts the COMBINED gz
(server+GUI), whose GUI can't make an OpenGL context headless and takes the
server down. So we start gz SERVER-ONLY (gz_args: '-s ... --headless-rendering')
and run only robot_spawn.launch.py. The fragile part was timing: if the robot is
spawned before gz finishes loading the (heavy) pipeline world, the model's
gz_ros2_control plugin intermittently fails to bring up a controller_manager →
no joint_states/odom/movement. So instead of `sleep N` we WAIT ON A CONDITION —
the gz world's spawn service appearing — then spawn. Sequencing is done with
launch event handlers.

gz is driven through ros_gz_sim/gz_sim.launch.py (not raw `gz sim`): that launch
sets GZ_SIM_SYSTEM_PLUGIN_PATH (from LD_LIBRARY_PATH + package exports, so the
ROS-installed gz_ros2_control system plugin in /opt/ros/jazzy/lib is found) and
appends model paths — subsuming the old explicit plugin-path/resource-path sets.
We still set GZ_SIM_RESOURCE_PATH ourselves (mirroring clearpath_gz's own
gz_sim.launch.py) so the pipeline world + its meshes load, and we still add the
/clock bridge ourselves because ros_gz_sim's launch does NOT provide one.

/clock: the gz→ROS clock bridge added below must be a LIVE node. If ROS /clock
shows 0 publishers and nodes block on "No clock received" (→ no odom/movement),
check `ros2 node list` for clock_bridge and do a clean restart — a stale/dead
bridge on a long-running container was the original #7 failure.
See docs/sim-debugging-notes.md #7.
"""
import json
import os
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (ExecuteProcess, IncludeLaunchDescription,
                            RegisterEventHandler, SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

WORLD = 'pipeline'
SETUP_PATH = '/clearpath/'

# External world bundles (WildSeed etc.): set SIM_WORLD=<bundle-name> (via
# deploy.sh world / .env) to load /worlds_external/<bundle>/world.sdf instead
# of the Clearpath pipeline world. A bundle is produced on the host by
# scripts/prepare_wildseed_world.sh and contains:
#   world.sdf    shell-injected world (Sensors/Imu/NavSat/… + spherical_coords)
#   models/      every model:// the world references (goes on the resource path)
#   spawn.json   {x, y, z, yaw, world_name} — z sampled from the terrain mesh
#                so the Husky spawns ON the (non-flat) terrain
WORLDS_EXTERNAL = '/worlds_external'


def _external_bundle():
    """Resolve SIM_WORLD -> (world_sdf, models_dir, world_name, spawn) or None."""
    bundle = os.environ.get('SIM_WORLD', '').strip()
    if not bundle:
        return None
    bdir = os.path.join(WORLDS_EXTERNAL, bundle)
    world_sdf = os.path.join(bdir, 'world.sdf')
    if not os.path.isfile(world_sdf):
        raise RuntimeError(
            f"SIM_WORLD='{bundle}' but {world_sdf} not found — is "
            f"worlds_external/ mounted and the bundle prepared? "
            f"(scripts/prepare_wildseed_world.sh)")
    with open(world_sdf, encoding='utf-8') as f:
        m = re.search(r'<world\s+name=["\']([^"\']+)["\']', f.read(65536))
    if not m:
        raise RuntimeError(f'no <world name="..."> in {world_sdf}')
    spawn = {}
    spawn_json = os.path.join(bdir, 'spawn.json')
    if os.path.isfile(spawn_json):
        with open(spawn_json, encoding='utf-8') as f:
            spawn = json.load(f)
    return world_sdf, os.path.join(bdir, 'models'), m.group(1), spawn


def generate_launch_description():
    cpg = get_package_share_directory('clearpath_gz')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    world_sdf = os.path.join(cpg, 'worlds', f'{WORLD}.sdf')
    world_name = WORLD
    extra_resources, spawn = [], {}

    # SLOW_SIM_FACTOR (from .env via compose x-env, default 1) multiplies the
    # WALL-clock ceilings below. On a slow sim (low RTF) the sim-time-paced
    # controller_manager needs proportionally more wall time to answer spawner/
    # service handshakes — the exact starvation documented in
    # docs/operations.md "Slow machines / low RTF".
    try:
        f = max(1, int(os.environ.get('SLOW_SIM_FACTOR', '1') or 1))
    except ValueError:
        f = 1

    external = _external_bundle()
    if external:
        world_sdf, models_dir, world_name, spawn = external
        extra_resources = [models_dir]

    # Resource path: the pipeline world references model://pipeline/* and
    # model://accessories/* meshes under clearpath_gz/meshes, plus models from
    # other sourced packages. Mirror clearpath_gz/launch/gz_sim.launch.py — put
    # worlds, meshes, and every sourced package's share/ on GZ_SIM_RESOURCE_PATH —
    # else gz logs "Failed to load a world" and silently skips model plugins.
    pkg_shares = [os.path.join(p, 'share')
                  for p in os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep) if p]
    # For external bundles, the bundle's models/ dir is prepended so the
    # world's model://ground, model://tree/... URIs resolve.
    set_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.pathsep.join([*extra_resources,
                         os.path.join(cpg, 'worlds'),
                         os.path.join(cpg, 'meshes'),
                         *pkg_shares,
                         os.environ.get('GZ_SIM_RESOURCE_PATH', '')]))

    # robot.yaml is mounted read-only at /clearpath-src; copy into the writable
    # setup_path the Clearpath generators write into.
    prepare = ExecuteProcess(
        cmd=['bash', '-c',
             f'mkdir -p {SETUP_PATH} && cp /clearpath-src/robot.yaml {SETUP_PATH}'],
        output='screen')

    # gz SERVER-ONLY + headless rendering, via ros_gz_sim's standard launch (which
    # sets GZ_SIM_SYSTEM_PLUGIN_PATH so libgz_ros2_control-system.so is found).
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': f'-s -r -v3 --headless-rendering {world_sdf}'}.items())

    # CONDITION (not a delay): poll until gz advertises the world's spawn service,
    # i.e. the world is loaded and ready to accept a model. Starts alongside gz and
    # exits as soon as ready (or after 180×f tries as a backstop — WildSeed worlds
    # include hundreds of mesh models and load slower than the pipeline world).
    wait_for_world = ExecuteProcess(
        cmd=['bash', '-c',
             f'for i in $(seq 1 {180 * f}); do '
             f'gz service -l 2>/dev/null | grep -q "/world/{world_name}/create" && exit 0; '
             f'sleep 1; done; exit 0'],
        output='screen')

    # gz→ROS /clock bridge. ros_gz_sim/gz_sim.launch.py does NOT add one (only
    # clearpath_gz's wrapper does), and without it every use_sim_time:=true node
    # (the gz_ros2_control controller_manager, robot_localization's ekf_node) blocks
    # on "No clock received" → no odom / no movement. This is Clearpath's canonical
    # bridge: plain /clock, '[gz.msgs.Clock' (GZ→ROS, subscriber-side on gz), no
    # remap, no use_sim_time override. See module docstring + sim-debugging-notes.md #7.
    clock_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen')

    # Spawn pose: on the flat pipeline world Clearpath's default z=0.15 works;
    # on external (WildSeed) terrain the bundle's spawn.json carries a z sampled
    # from the terrain mesh at (x, y) so the Husky starts ON the ground.
    spawn_args = {'world': world_name, 'setup_path': SETUP_PATH,
                  'use_sim_time': 'true'}
    for k in ('x', 'y', 'z', 'yaw'):
        if k in spawn:
            spawn_args[k] = str(spawn[k])

    robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(cpg, 'launch', 'robot_spawn.launch.py')),
        launch_arguments=spawn_args.items())

    # Heavy (WildSeed) worlds dip the RTF hard while their meshes load, and a
    # controller's first sim-time-paced activation can starve ("Failed to
    # activate joint_state_broadcaster", spawner dies) even though a later
    # attempt succeeds. Idempotent watchdog: poll the controller list and
    # re-spawn/re-activate anything not active; exits as soon as both are
    # active. No-op on the flat pipeline world (they activate first try).
    #
    # Clearpath's robot_spawn.launch.py runs the INITIAL spawners with their
    # default wall-clock --controller-manager-timeout (not widenable from here);
    # at low RTF those may die — this watchdog is their designed recovery. All
    # wall budgets below scale with SLOW_SIM_FACTOR (f); the spawner's
    # controller-manager handshake is the timeout that actually starves.
    cm = '/a200_0000/controller_manager'
    controller_watchdog = ExecuteProcess(
        cmd=['bash', '-c',
             'source /opt/ros/jazzy/setup.bash; '
             f'for i in $(seq 1 {30 * f}); do sleep 15; '
             f'  L=$(timeout {10 * f} ros2 service call ' + cm + '/list_controllers '
             '      controller_manager_msgs/srv/ListControllers 2>/dev/null); '
             '  ok=1; '
             '  for c in joint_state_broadcaster platform_velocity_controller; do '
             '    echo "$L" | grep -q "name=.$c., state=.active." && continue; '
             '    ok=0; '
             '    echo "[controller_watchdog] $c not active — recovering"; '
             f'    timeout {40 * f} ros2 run controller_manager spawner "$c" '
             f'      --controller-manager-timeout {20 * f} --ros-args -r __ns:=/a200_0000 '
             '      2>/dev/null || '
             f'    timeout {10 * f} ros2 service call ' + cm + '/switch_controller '
             '      controller_manager_msgs/srv/SwitchController '
             '      "{activate_controllers: [$c], strictness: 1}" >/dev/null 2>&1; '
             '  done; '
             '  [ "$ok" = 1 ] && { echo "[controller_watchdog] all active"; exit 0; }; '
             'done; '
             'echo "[controller_watchdog] gave up — check list_controllers; '
             'if RTF < 0.1 raise SLOW_SIM_FACTOR in .env (docs/operations.md)" >&2'],
        output='screen')

    # Info-only RTF probe: after bring-up settles, report the measured real-time
    # factor in the launch log and WARN below 0.1 (where wall-clock handshakes
    # start starving). Never fails the launch (always exit 0).
    rtf_probe = ExecuteProcess(
        cmd=['bash', '-c',
             'source /opt/ros/jazzy/setup.bash; sleep 45; '
             f'r=$(gz topic -e -n 5 -t /world/{world_name}/stats 2>/dev/null '
             '  | grep -oE "real_time_factor: [0-9.eE+-]+" '
             '  | awk \'{s+=$2;c++} END {if (c) printf "%.3f", s/c}\'); '
             'if [ -z "$r" ]; then echo "[rtf_probe] RTF unavailable (stats topic silent)"; exit 0; fi; '
             'echo "[rtf_probe] RTF≈$r"; '
             'awk -v r="$r" \'BEGIN { if (r < 0.1) print "[rtf_probe] WARN: RTF < 0.1 — '
             'controller activation may starve; set SLOW_SIM_FACTOR in .env and see '
             'docs/operations.md (Slow machines / low RTF)" }\'; exit 0'],
        output='screen')

    return LaunchDescription([
        set_resource_path,
        prepare,
        # prepare done -> start gz (via ros_gz_sim) and begin polling for world-ready
        RegisterEventHandler(OnProcessExit(target_action=prepare,
                                          on_exit=[gz_sim, wait_for_world])),
        # world ready -> start the /clock bridge, then spawn robot + controllers
        # (+ the activation watchdog for heavy worlds + the info-only RTF probe)
        RegisterEventHandler(OnProcessExit(target_action=wait_for_world,
                                          on_exit=[clock_bridge, robot_spawn,
                                                   controller_watchdog, rtf_probe])),
    ])
