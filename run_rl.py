"""Script for running reinforcement learning on the drone forest."""

import os

from stable_baselines3 import PPO

from stable_baselines3.common.callbacks import EvalCallback
from stable_baselines3.common.utils import set_random_seed
from stable_baselines3.common.vec_env import SubprocVecEnv

from drone_forest.gym_wrapper import DroneForestEnv


def make_env(rank: int, seed: int = 0) -> DroneForestEnv:
    """Make the drone forest environment."""

    def _init() -> DroneForestEnv:
        env = DroneForestEnv(
            dt=0.1,
            xlim=(-15, 15),
            ylim=(-3, 27),
            n_trees=100,
            max_tree_radius=0.7,
            n_lidar_beams=36,
            max_lidar_range=3.0,
            min_spare_distance=0.5,
            max_spawn_attempts=50,
            seed=seed + rank,
        )
        env.reset(seed=seed + rank)
        return env

    set_random_seed(seed)
    return _init


if __name__ == "__main__":
    # Set the parameters for the simulation
    n_training_envs = 8
    n_eval_envs = 4
    n_learning_steps = 10_000_000
    n_eval_every = 100_000

    # Create log dir where evaluation results will be saved
    eval_log_dir = "./eval_logs/"
    os.makedirs(eval_log_dir, exist_ok=True)

    # Create the environments
    train_envs = SubprocVecEnv([make_env(i) for i in range(n_training_envs)])
    eval_envs = SubprocVecEnv([make_env(i) for i in range(n_eval_envs)])

    eval_callback = EvalCallback(
        eval_envs,
        best_model_save_path=eval_log_dir,
        log_path=eval_log_dir,
        eval_freq=max(n_eval_every // n_training_envs, 1),
        n_eval_episodes=1,
        deterministic=True,
        render=True,
    )

    # Create the reinforcement learning model
    model = PPO("MlpPolicy", train_envs, verbose=1)
    model.learn(total_timesteps=n_learning_steps, callback=eval_callback)
