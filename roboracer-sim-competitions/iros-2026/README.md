# IROS 2026 online sim racing

Penn RoboRacer stack for the AutoDRIVE RoboRacer Sim-Racing League (IROS 2026, online). We race **pure pursuit** (`pp_backup_node`) on a mapped raceline, localized with **Nav2 AMCL** (particle filter). Kinematic MPC stays in the image as a one-argument swap. The default map and raceline are a full copy of the playground save **track-20260731**. The controller, AMCL, and localization helper modules are copied into this folder (not imported from `roboracer-playground`).

The organizers provide the Unity simulator, the ROS 2 Socket.IO bridge, and the Docker submission pattern. This folder is our racing stack on top of that: algorithms, launch files, maps, and waypoints.

```
Simulator (host)  --Socket.IO :4567-->  Docker (ROS 2 Humble + AMCL + PP/MPC)
```

## Prerequisites

- [Docker Desktop](https://www.docker.com/products/docker-desktop/) (or Docker Engine + Compose)
- The AutoDRIVE **practice** simulator for your OS (not in this folder)

Node.js is not required to build or run.

## 1. Get the simulator

1. Open the [IROS 2026 release](https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE-RoboRacer-Sim-Racing/releases/tag/2026-iros).
2. Download `autodrive_simulator_practice_XXX.zip` for your OS.
3. Extract it anywhere (for example next to this folder).

**macOS only** — grant execute permission and drop the quarantine flag:

```bash
cd /path/to/extracted/autodrive_simulator
sudo chmod -R +x "AutoDRIVE Simulator.app/Contents/MacOS"
xattr -d com.apple.quarantine "AutoDRIVE Simulator.app"
```

## 2. Build the container

From this `iros-2026/` directory:

```bash
docker build --platform linux/amd64 \
  --tag penn-roboracer/iros-2026-api:latest \
  -f autodrive_devkit.Dockerfile .
```

Use `--platform linux/amd64` on Apple Silicon. Omit it on native Linux/amd64.

The image is ROS 2 Humble. It already includes `foxglove-bridge`, `slam-toolbox`, Nav2 AMCL, and the CVXPY/OSQP MPC solver.

## 3. Run the container

**macOS / Windows:** map ports (`-p 4567:4567`). The official organizer docs use `--network=host`, which is Linux-only.

Do not launch `bringup_headless.launch.py` in the same container as the default entrypoint — the entrypoint already starts the race stack.

### Race (matches the submission entrypoint)

This starts `race.launch.py`: AutoDRIVE bridge, map_server + AMCL on `track-20260731`, `/initialpose` seeding with that map’s `map_to_world`, and pure pursuit (`pp_backup_node`).

```bash
docker run --platform linux/amd64 --name iros-2026-api --rm -it \
  -p 4567:4567 -p 8765:8765 \
  penn-roboracer/iros-2026-api:latest
```

To run kinematic MPC instead (same image, no rebuild):

```bash
docker run --platform linux/amd64 --name iros-2026-api --rm -it \
  -p 4567:4567 -p 8765:8765 \
  penn-roboracer/iros-2026-api:latest \
  controller:=mpc
```

### Dev (edit code without rebuilding)

Overrides the entrypoint and mounts this folder’s `autodrive_devkit/` into the container:

```bash
docker run --platform linux/amd64 --name iros-2026-api --rm -it \
  -p 4567:4567 -p 8765:8765 \
  -v "$(pwd)/autodrive_devkit":/home/autodrive_devkit/src/autodrive_devkit \
  --entrypoint /bin/bash \
  penn-roboracer/iros-2026-api:latest
```

Inside the container, rebuild if you changed Python/launch files, then start the stack:

```bash
cd /home/autodrive_devkit && colcon build && source install/setup.bash
ros2 launch autodrive_roboracer race.launch.py
```

Swap back to kinematic MPC (no file edits):

```bash
ros2 launch autodrive_roboracer race.launch.py controller:=mpc
```

Override map or raceline:

```bash
ros2 launch autodrive_roboracer race.launch.py \
  map:=/home/autodrive_devkit/src/autodrive_devkit/maps/track-20260731/map.yaml \
  waypoint_file:=/home/autodrive_devkit/src/autodrive_devkit/racelines/track-20260731-line/waypoints.csv \
  map_name:=track-20260731
```

Bridge only (no racing algorithm):

```bash
ros2 launch autodrive_roboracer bringup_headless.launch.py
```

## 4. Open the simulator

1. Launch the practice app (on Mac, double-click `AutoDRIVE Simulator.app`).
2. Set IP `127.0.0.1`, port `4567`, then **Connect**.
3. Wait for `Connected!` in the Docker terminal.
4. Switch the vehicle to **Autonomous**.

AMCL seeds from current odometry transformed by the copied map’s `map_to_world` (`maps/track-20260731/meta.json`, via the copied `store.py` / `localize.py`). Pure pursuit holds zero throttle until `/amcl_pose` is live.

## 5. Optional: Foxglove

1. Open [app.foxglove.dev](https://app.foxglove.dev/).
2. **Open Connection** → Foxglove WebSocket → `ws://localhost:8765`.
3. Import the layout `foxglove/AutoDRIVE RoboRacer Sim Racing League.json`.

If Foxglove is not already running (dev / bringup-only), exec into the container and start it:

```bash
docker exec -it iros-2026-api bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

## What we race

| Piece | Path |
|-------|------|
| Controller (default) | `autodrive_devkit/racing_algo/pp_backup_node.py` (playground pure pursuit) |
| PP defaults | `autodrive_devkit/pp_backup_params.yaml` (playground UI sliders, 31 Jul tune) |
| Alternate | `autodrive_devkit/racing_algo/mpc_node.py` + `kinematic_mpc.py` (`controller:=mpc`) |
| MPC defaults | `autodrive_devkit/mpc_params.yaml` (playground UI sliders) |
| Localization | Nav2 AMCL via `autodrive_devkit/launch/localize.launch.py` (full copy) |
| Localize helpers | `autodrive_devkit/localize.py`, `autodrive_devkit/store.py` (full copies) |
| Pose seed | `autodrive_devkit/racing_algo/amcl_initialpose_node.py` |
| Map | `autodrive_devkit/maps/track-20260731/` |
| Raceline | `autodrive_devkit/racelines/track-20260731-line/waypoints.csv` |
| Launch | `autodrive_devkit/launch/race.launch.py` (`controller:=pp_backup` default) |

Pure pursuit tracks `/amcl_pose` for path pose and odometry twist for speed. MPC (`controller:=mpc`) uses `/amcl_pose` for both pose and speed (finite-difference of successive poses). Both hold zero throttle until `/amcl_pose` is live. Odometry also seeds AMCL at start.

Swap controllers without rebuilding or editing files:

```bash
ros2 launch autodrive_roboracer race.launch.py                    # pp_backup
ros2 launch autodrive_roboracer race.launch.py controller:=mpc    # kinematic MPC
```

## Submission

Evaluators pull a Docker Hub image and run it with `--network=host` next to **their** simulator container. We submit only the **devkit** image, not the simulator.

Typical organizer run (Linux):

```bash
docker run --name autodrive_roboracer_api --rm -it \
  --network=host --ipc=host \
  --entrypoint /bin/bash \
  <TEAM_USERNAME>/<IMAGE_NAME>:<TAG>
```

Our image’s default entrypoint already launches `race.launch.py` with **pure pursuit**. If they override `--entrypoint /bin/bash`, start the race with `ros2 launch autodrive_roboracer race.launch.py`. To race MPC instead, pass `controller:=mpc` after the image name or as a launch argument.

## Appendix: remake the map

The race stack does **not** run SLAM while racing. Use this only to rebuild a map and raceline. After remapping, AMCL loads the occupancy grid and the controller follows waypoints in that map frame.

1. Start the container in **dev** mode and launch `bringup_headless.launch.py` (not `race.launch.py`).
2. Connect the simulator in **Manual** mode.
3. In a second container shell:

```bash
ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
  --params-file /home/autodrive_devkit/src/autodrive_devkit/f1tenth_online_async.yaml \
  -r __node:=slam_toolbox
```

4. Drive one slow lap. Save:

```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: 'practice_track'}}"
```

5. Copy `practice_track.yaml` / `practice_track.pgm` into the mounted `autodrive_devkit/`, then generate waypoints:

```bash
cd autodrive_devkit
python3 generate_waypoints.py --map practice_track.yaml --out practice_waypoints.csv
```

Or open `raceline_explorer_gui.py` / `waypoint_editor_gui.py`.

If the saved map is offset from the simulator world frame, keep `map_to_world` in the map `meta.json` (copied with the map) or pass a different `map_name` so `store.map_meta` loads that transform.

## Citation

The bridge, Docker pattern, and simulator are part of the [AutoDRIVE Ecosystem](https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE-RoboRacer-Sim-Racing). If you use that framework, cite:

- Samak et al., “AutoDRIVE: A Comprehensive, Flexible and Integrated Digital Twin Ecosystem…”, *MDPI Robotics*, 2023. [doi:10.3390/robotics12030077](https://doi.org/10.3390/robotics12030077)
- Samak et al., “AutoDRIVE Simulator…”, CCRIS 2021. [doi:10.1145/3483845.3483846](https://doi.org/10.1145/3483845.3483846)
