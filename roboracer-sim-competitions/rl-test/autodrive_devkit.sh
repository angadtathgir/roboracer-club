#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /home/autodrive_devkit/install/setup.bash
export AUTODRIVE_RACE_ARTIFACT_DIR="${AUTODRIVE_RACE_ARTIFACT_DIR:-/home/autodrive_devkit/src/autodrive_race}"

cd /home/autodrive_devkit

# Bridge + Foxglove only. Start PPO training in a second shell with:
#   ros2 run autodrive_race ppo_training -- --timesteps 20000 --checkpoint-freq 5000
ros2 launch autodrive_roboracer bringup_headless.launch.py
