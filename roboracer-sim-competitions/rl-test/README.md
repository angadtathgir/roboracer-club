# RL test (PPO on AutoDRIVE)

Penn RoboRacer copy of [f1tenth-rl-controller](https://github.com/1Kaustubh122/f1tenth-rl-controller) plus the AutoDRIVE ROS 2 **bridge** (from `icra-2026`). This folder is for trying PPO in the practice simulator. It is not a competition submission stack.

```
Simulator (host)  --Socket.IO :4567-->  Docker (bridge + PPO trainer)
```

The trainer reads LiDAR / collisions / laps and publishes throttle + steering. There is **no** Pure Pursuit or MPC running here — those would fight the policy for the same command topics.

Two policy paths live in `autodrive_race/`:

| Path | Command | Observation | Action |
|------|---------|-------------|--------|
| Baseline (start here) | `ppo_training` | 180 LiDAR bins | `Discrete(25)` |
| Advanced (train from scratch) | `final_test_training` | 240 LiDAR + 2 prev-action | `MultiDiscrete([4, 11])` |

## Prerequisites

- [Docker Desktop](https://www.docker.com/products/docker-desktop/)
- AutoDRIVE **practice** simulator (ICRA 2026 or IROS 2026 release — same `/autodrive/roboracer_1/...` topics)

## 1. Get the simulator

Same as `icra-2026/` or `iros-2026/`: download the practice zip, extract it, and on macOS:

```bash
cd /path/to/extracted/autodrive_simulator
sudo chmod -R +x "AutoDRIVE Simulator.app/Contents/MacOS"
xattr -d com.apple.quarantine "AutoDRIVE Simulator.app"
```

## 2. Build the container

From this `rl-test/` directory:

```bash
docker build --platform linux/amd64 \
  --tag penn-roboracer/rl-test:latest \
  -f autodrive_devkit.Dockerfile .
```

Use `--platform linux/amd64` on Apple Silicon. The image is ROS 2 Humble plus Gymnasium, Stable-Baselines3, CPU PyTorch, and TensorBoard.

First build downloads PyTorch and will take a while.

## 3. Run the container (bridge)

**macOS / Windows:** map ports. `--network=host` is Linux-only.

```bash
docker run --platform linux/amd64 --name rl-test --rm -it \
  -p 4567:4567 -p 8765:8765 -p 6006:6006 \
  -v "$(pwd)/autodrive_race":/home/autodrive_devkit/src/autodrive_race \
  -v "$(pwd)/autodrive_devkit":/home/autodrive_devkit/src/autodrive_devkit \
  penn-roboracer/rl-test:latest
```

The bind mounts keep checkpoints and TensorBoard logs on your Mac in `autodrive_race/`.

This starts the AutoDRIVE bridge + Foxglove. It does **not** start training.

### Dev shell (no auto-launch)

```bash
docker run --platform linux/amd64 --name rl-test --rm -it \
  -p 4567:4567 -p 8765:8765 -p 6006:6006 \
  -v "$(pwd)/autodrive_race":/home/autodrive_devkit/src/autodrive_race \
  -v "$(pwd)/autodrive_devkit":/home/autodrive_devkit/src/autodrive_devkit \
  --entrypoint /bin/bash \
  penn-roboracer/rl-test:latest
```

Inside:

```bash
cd /home/autodrive_devkit && colcon build --symlink-install && source install/setup.bash
export AUTODRIVE_RACE_ARTIFACT_DIR=/home/autodrive_devkit/src/autodrive_race
ros2 launch autodrive_roboracer bringup_headless.launch.py
```

## 4. Open the simulator

1. Launch the practice app.
2. Set IP `127.0.0.1`, port `4567`, then **Connect**.
3. Wait for `Connected!` in the Docker terminal.
4. Switch the vehicle to **Autonomous**.

Confirm LiDAR is flowing (second container shell):

```bash
docker exec -it rl-test bash
source /opt/ros/humble/setup.bash
source /home/autodrive_devkit/install/setup.bash
ros2 topic hz /autodrive/roboracer_1/lidar
```

You want a non-zero rate. Also check `/autodrive/roboracer_1/collision_count` and `/autodrive/roboracer_1/lap_count`.

## 5. Train (baseline smoke test)

In a **second** container shell:

```bash
docker exec -it rl-test bash
source /opt/ros/humble/setup.bash
source /home/autodrive_devkit/install/setup.bash
export AUTODRIVE_RACE_ARTIFACT_DIR=/home/autodrive_devkit/src/autodrive_race

ros2 run autodrive_race ppo_training -- --timesteps 20000 --checkpoint-freq 5000
```

Use a small `--timesteps` first. Default is 1,000,000, which is many hours at ~10 env steps/sec.

The car should start twitching (random-ish steering) as soon as training starts. If it sits still, LiDAR/commands are not connected.

Ctrl+C saves a resumable checkpoint. Re-running the same command resumes from the latest compatible zip in `autodrive_race/checkpoints/`.

### Full baseline run

```bash
ros2 run autodrive_race ppo_training -- --timesteps 1000000 --checkpoint-freq 10000
```

### Advanced (from scratch, different observation/action contract)

```bash
ros2 run autodrive_race final_test_training -- \
  --device cpu \
  --timesteps 3000000 \
  --n-steps 4096 \
  --batch-size 512 \
  --checkpoint-freq 10000
```

On a Linux box with an NVIDIA GPU, pass `--device cuda` instead of `cpu`.

## 6. Is it actually training?

Yes — you can see it without waiting for a “good” policy.

### A. The car (fastest sanity check)

In the simulator the car should move immediately. At the start it will crash a lot. That is expected: reward is `+1` per step plus a small speed bonus, and `-50` (baseline) on collision. Early episodes are short.

### B. Per-episode lines (every crash or 12 laps)

Look for:

```text
episode 12  reward=8.2  mean20=6.1  steps=9  crash=True  collisions=12  laps=0  ...
```

| Field | Healthy early training | Stuck / not learning |
|-------|------------------------|----------------------|
| `reward` / `mean20` | noisy, slowly rising over hundreds of episodes | stuck near 0, or never prints |
| `steps` (episode length) | getting longer (survives more steps before a crash) | always 1–5 steps forever |
| `crash` | True a lot at first, less often later | True every episode with no change |
| `laps` / `last_lap` | eventually non-zero | always 0 after a long time |
| `total_timesteps` | climbing | frozen |

If you never see an `episode ...` line, the env is not stepping (usually no ROS connection).

### C. PPO dump (every `n_steps` env steps)

Baseline uses `n_steps=2048`. Each step waits up to 0.1 s on ROS, so the **first table is ~3–4 minutes**, not instant.

You want a table like:

```text
-----------------------------------------
| rollout/ep_len_mean          | 12.4   |
| rollout/ep_rew_mean          | 7.8    |
| train/entropy_loss           | -3.1   |
| train/policy_gradient_loss   | -0.01  |
| train/value_loss             | 8.4    |
| train/approx_kl              | 0.008  |
| time/fps                     | 8      |
-----------------------------------------
```

| Metric | Meaning | What “training” looks like |
|--------|---------|----------------------------|
| `rollout/ep_rew_mean` | mean episode return | the number to watch; should trend up over hours |
| `rollout/ep_len_mean` | mean steps before crash/timeout | should trend up as it stops crashing immediately |
| `train/approx_kl` | policy changed this update | should be small but **not always 0** |
| `train/entropy_loss` | how random the policy is | magnitude usually shrinks as it gets less random |
| `train/policy_gradient_loss` / `value_loss` | optimizer is running | non-zero, value loss often falls then plateaus |
| `time/fps` | env steps per second | ~5–10 in this ROS loop; 0 means hung |

Random exploration still “trains” (weights update). Learning **progress** is the reward/length trend, not a single dump.

### D. TensorBoard

From a third container shell (or the same one after starting training):

```bash
tensorboard --logdir /home/autodrive_devkit/src/autodrive_race/logs/tb --bind_all --port 6006
```

On the host open [http://localhost:6006](http://localhost:6006).

Curves to watch:

- `rollout/ep_rew_mean`, `rollout/ep_len_mean`
- `episode/reward`, `episode/mean_reward_20`, `episode/length` (from our callback)
- `episode/collision` (should drift down)
- `train/*` (proves gradient updates)

If TensorBoard is empty, training has not completed a PPO update yet (wait for that first 2048-step dump).

### E. Files on disk

With the bind mount, on the host:

```text
autodrive_race/checkpoints/ppo_model_<steps>_steps.zip
autodrive_race/logs/tb/baseline_ppo_1/
autodrive_race/ppo_race_model_final.zip   # only after a full run finishes
```

If zips appear and grow, checkpoints are working. Baseline default checkpoint spacing is 10k steps (~15–20 min).

## 7. Evaluate a saved model

Keep the bridge + simulator running, stop training, then:

```bash
ros2 run autodrive_race eval -- --model-path /home/autodrive_devkit/src/autodrive_race/checkpoints/ppo_model_20000_steps.zip --steps 2000
```

The evaluator picks baseline vs advanced from the zip’s observation/action contract.

## 8. Optional: Foxglove

1. [app.foxglove.dev](https://app.foxglove.dev/) → Open Connection → `ws://localhost:8765`
2. Import `foxglove/AutoDRIVE RoboRacer Sim Racing League.json`

## Notes

- **Single simulator:** do not pass `--eval` unless you have a second sim. Train and eval would both publish throttle/steering.
- **No car reset in the env:** `reset()` only clears counters. Rely on the simulator’s collision respawn. If the car gets stuck, reset it in the sim UI.
- **Apple Silicon:** this image is `linux/amd64` under emulation. Training works for a smoke test but is slow. Use a Linux/NVIDIA machine for a serious run.
- ONNX / TensorRT / Triton scripts are copied through but not needed to train or eval.

## Citation

RL package: MIT, Kaustubh Krishna (`LICENSE`). Bridge/simulator: AutoDRIVE Ecosystem (see `icra-2026/README.md`).
