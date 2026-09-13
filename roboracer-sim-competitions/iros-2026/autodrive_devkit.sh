#!/bin/bash
set -e

# Setup Development Environment
source /opt/ros/humble/setup.bash
source /home/autodrive_devkit/install/setup.bash

# Copied playground store.py reads maps/ racelines/ from this root
export PLAYGROUND_DATA_DIR="${PLAYGROUND_DATA_DIR:-/home/autodrive_devkit/src/autodrive_devkit}"

# AutoDRIVE Devkit Workspace
cd /home/autodrive_devkit

# Launch AutoDRIVE Devkit with GUI
# ros2 launch autodrive_roboracer bringup_graphics.launch.py

# Launch AutoDRIVE Devkit Headless
# ros2 launch autodrive_roboracer bringup_headless.launch.py

# Race stack. Default controller is pp_backup (pure pursuit on track-20260731).
# Extra args are forwarded to race.launch.py, so swap back to MPC with:
#   docker run ... penn-roboracer/iros-2026-api:latest controller:=mpc
# or inside the container:
#   ros2 launch autodrive_roboracer race.launch.py controller:=mpc
ros2 launch autodrive_roboracer race.launch.py "$@"