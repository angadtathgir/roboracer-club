"""Baseline PPO training entry point."""

from __future__ import annotations

import argparse
import logging
from pathlib import Path

from autodrive_race.constants import DEFAULT_CHECKPOINT_FREQ, DEFAULT_TOTAL_TIMESTEPS
from autodrive_race.utils import (
    ensure_directories,
    find_latest_checkpoint,
    find_latest_compatible_checkpoint,
    get_artifact_root,
)

LOGGER = logging.getLogger(__name__)


def parse_args(argv: list[str] | None = None) -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser(description="Resume or start baseline PPO training.")
    parser.add_argument("--timesteps", type=int, default=DEFAULT_TOTAL_TIMESTEPS)
    parser.add_argument("--checkpoint-freq", type=int, default=DEFAULT_CHECKPOINT_FREQ)
    parser.add_argument(
        "--eval",
        action="store_true",
        help="Run periodic eval on a second env. Skip this on a single simulator "
        "(train and eval would fight over the same throttle/steering topics).",
    )
    return parser.parse_known_args(argv)


def build_model(env, checkpoint_path: Path | None, tensorboard_log: Path):
    from stable_baselines3 import PPO

    if checkpoint_path is not None and checkpoint_path.exists():
        LOGGER.info("Resuming baseline training from checkpoint: %s", checkpoint_path)
        return PPO.load(
            str(checkpoint_path),
            env=env,
            device="cpu",
            tensorboard_log=str(tensorboard_log),
        )

    LOGGER.info("No compatible baseline checkpoint found; starting a new baseline run.")
    return PPO(
        "MlpPolicy",
        env,
        verbose=1,
        learning_rate=3e-4,
        batch_size=256,
        n_steps=2048,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        ent_coef=0.01,
        policy_kwargs={"net_arch": [256, 256]},
        device="cpu",
        tensorboard_log=str(tensorboard_log),
    )


def main(args=None) -> None:
    cli_args, ros_args = parse_args(args)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")

    import rclpy
    from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback
    from stable_baselines3.common.env_util import make_vec_env

    from autodrive_race.callbacks import EpisodeMetricsCallback
    from autodrive_race.env import make_baseline_env

    rclpy.init(args=ros_args)
    train_env = None
    eval_env = None
    model = None
    try:
        package_root = get_artifact_root()
        checkpoint_dir = package_root / "checkpoints"
        best_model_dir = package_root / "best_model"
        logs_dir = package_root / "logs"
        tb_dir = logs_dir / "tb"
        ensure_directories(checkpoint_dir, best_model_dir, logs_dir, tb_dir)

        latest_checkpoint, latest_steps = find_latest_compatible_checkpoint(
            checkpoint_dir,
            lambda contract: contract.is_baseline,
        )
        if latest_checkpoint is not None:
            LOGGER.info("Latest compatible baseline checkpoint: %s (%d steps)", latest_checkpoint.name, latest_steps)
        else:
            incompatible_checkpoint, incompatible_steps = find_latest_checkpoint(checkpoint_dir)
            if incompatible_checkpoint is not None:
                LOGGER.warning(
                    "Ignoring incompatible checkpoint %s (%d steps); it does not match the "
                    "baseline saved-model contract.",
                    incompatible_checkpoint.name,
                    incompatible_steps,
                )

        train_env = make_vec_env(lambda: make_baseline_env(train=True), n_envs=1)
        model = build_model(train_env, latest_checkpoint, tb_dir)
        resuming = latest_checkpoint is not None and latest_checkpoint.exists()

        callbacks = [
            EpisodeMetricsCallback(),
            CheckpointCallback(
                save_freq=cli_args.checkpoint_freq,
                save_path=str(checkpoint_dir),
                name_prefix="ppo_model",
            ),
        ]
        if cli_args.eval:
            eval_env = make_vec_env(lambda: make_baseline_env(train=False), n_envs=1)
            callbacks.append(
                EvalCallback(
                    eval_env,
                    best_model_save_path=str(best_model_dir),
                    log_path=str(logs_dir),
                    eval_freq=cli_args.checkpoint_freq,
                    deterministic=True,
                )
            )
        else:
            LOGGER.info(
                "EvalCallback disabled (single-sim default). Pass --eval to enable it. "
                "Watch episode logs, the PPO table every %d steps, TensorBoard at %s, "
                "and checkpoints in %s.",
                model.n_steps,
                tb_dir,
                checkpoint_dir,
            )

        try:
            model.learn(
                total_timesteps=cli_args.timesteps,
                callback=callbacks,
                progress_bar=True,
                reset_num_timesteps=not resuming,
                tb_log_name="baseline_ppo",
            )
        except KeyboardInterrupt:
            interrupt_checkpoint = checkpoint_dir / f"ppo_model_{int(model.num_timesteps)}_steps.zip"
            model.save(str(interrupt_checkpoint))
            LOGGER.warning("Training interrupted; saved resumable checkpoint to %s", interrupt_checkpoint)
            raise

        final_model_path = package_root / "ppo_race_model_final.zip"
        model.save(str(final_model_path))
        LOGGER.info("Final baseline model saved to %s", final_model_path)
    finally:
        if eval_env is not None:
            eval_env.close()
        if train_env is not None:
            train_env.close()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
