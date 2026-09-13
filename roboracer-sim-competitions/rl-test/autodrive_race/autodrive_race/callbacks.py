"""Training callbacks that print episode stats so you can see learning live."""

from __future__ import annotations

import logging

from stable_baselines3.common.callbacks import BaseCallback

LOGGER = logging.getLogger(__name__)


class EpisodeMetricsCallback(BaseCallback):
    """Log per-episode reward, length, collisions, and laps to stdout and TensorBoard."""

    def __init__(self, verbose: int = 1) -> None:
        super().__init__(verbose)
        self._episodes = 0
        self._recent_rewards: list[float] = []
        self._window = 20

    def _on_step(self) -> bool:
        infos = self.locals.get("infos", [])
        dones = self.locals.get("dones", [])
        for info, done in zip(infos, dones):
            if not done:
                continue

            episode = info.get("episode") or {}
            reward = float(episode.get("r", 0.0))
            length = int(episode.get("l", 0))
            self._episodes += 1
            self._recent_rewards.append(reward)
            if len(self._recent_rewards) > self._window:
                self._recent_rewards.pop(0)
            mean_reward = sum(self._recent_rewards) / len(self._recent_rewards)

            collision = bool(info.get("collision", False))
            collision_count = info.get("collision_count")
            lap_count = info.get("lap_count")
            last_lap_time = info.get("last_lap_time")
            best_lap_time = info.get("best_lap_time")

            self.logger.record("episode/reward", reward)
            self.logger.record("episode/length", length)
            self.logger.record("episode/mean_reward_20", mean_reward)
            self.logger.record("episode/collision", float(collision))
            if last_lap_time is not None:
                self.logger.record("episode/last_lap_time", float(last_lap_time))

            LOGGER.info(
                "episode %d  reward=%.1f  mean20=%.1f  steps=%d  crash=%s  "
                "collisions=%s  laps=%s  last_lap=%s  best_lap=%s  total_timesteps=%d",
                self._episodes,
                reward,
                mean_reward,
                length,
                collision,
                collision_count,
                lap_count,
                last_lap_time,
                best_lap_time,
                self.num_timesteps,
            )
        return True
