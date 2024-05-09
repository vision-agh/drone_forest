"""Script for running reinforcement learning on the drone forest."""

import json
import os
import time

from typing import Dict

from stable_baselines3 import PPO

from stable_baselines3.common.callbacks import EvalCallback
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.utils import set_random_seed
from stable_baselines3.common.vec_env import SubprocVecEnv

import torch

from scripts.gym_wrapper import DroneForestEnv


def make_env(rank: int, seed: int = 0, config_dict: Dict = {}) -> DroneForestEnv:
    """Make the drone forest environment."""

    def _init() -> DroneForestEnv:
        env = DroneForestEnv(
            actions=config_dict["actions"],
            dt=config_dict["sim_step"],
            x_lim=(config_dict["x_lim"]["min"], config_dict["x_lim"]["max"]),
            y_lim=(config_dict["y_lim"]["min"], config_dict["y_lim"]["max"]),
            y_static_limit=config_dict["y_static_limit"],
            goal_y=config_dict["y_lim"]["max"] - 2.0,
            n_trees=config_dict["n_trees"],
            tree_radius_lim=(
                config_dict["tree_radius_lim"]["min"],
                config_dict["tree_radius_lim"]["max"],
            ),
            n_lidar_beams=config_dict["n_lidar_beams"],
            lidar_range=config_dict["lidar_range"],
            min_tree_spare_distance=config_dict["min_tree_spare_distance"],
            max_spawn_attempts=config_dict["max_spawn_attempts"],
            max_speed=config_dict["max_speed"],
            max_acceleration=config_dict["max_acceleration"],
            drone_width_m=config_dict["drone_width"],
            drone_height_m=config_dict["drone_height"],
        )
        env.reset(seed=seed + rank)
        return env

    set_random_seed(seed)
    return _init


if __name__ == "__main__":
    # Set the parameters for the simulation
    n_training_envs = 16
    n_eval_envs = 4
    n_learning_steps = 100_000_000
    n_eval_every = 250_000

    # Read environment configuration
    with open("./env_config.json", "r") as config_file:
        config_dict = json.load(config_file)
    assert config_dict["nb_directions"] == 4, "Only 4 directions are supported."
    assert (
        config_dict["nb_actions"] % config_dict["nb_directions"] == 0
    ), "The number of actions must be a multiple of the number of directions."

    # Create action vectors based on the directions and number of actions
    base_actions_x = [0.0, -1.0, 0.0, 1.0]
    base_actions_y = [1.0, 0.0, -1.0, 0.0]
    actions = []
    actions_per_direction = config_dict["nb_actions"] // config_dict["nb_directions"]
    for idx_dir in range(config_dict["nb_directions"]):
        actions.extend(
            [
                [
                    base_actions_x[idx_dir] * (i + 1) / actions_per_direction,
                    base_actions_y[idx_dir] * (i + 1) / actions_per_direction,
                ]
                for i in range(actions_per_direction)
            ]
        )
    config_dict["actions"] = actions

    # Check gym wrapper
    check_env(make_env(0, config_dict=config_dict)())

    # Create log dir where evaluation results will be saved
    exp_dir = str(int(time.time()))
    log_dir = os.path.join("./logs_ppo", exp_dir)
    os.makedirs(log_dir, exist_ok=True)

    # Preserve the environment configuration
    with open(os.path.join(log_dir, "env_config.json"), "w") as config_file:
        json.dump(config_dict, config_file)

    # Create the environments
    train_envs = SubprocVecEnv(
        [make_env(i, config_dict=config_dict) for i in range(n_training_envs)]
    )
    eval_envs = SubprocVecEnv(
        [make_env(i, config_dict=config_dict) for i in range(n_eval_envs)]
    )

    eval_callback = EvalCallback(
        eval_envs,
        best_model_save_path=log_dir,
        log_path=log_dir,
        eval_freq=max(n_eval_every // n_training_envs, 1),
        n_eval_episodes=1,
        deterministic=True,
        render=False,
    )

    # Create the reinforcement learning model
    model = PPO(
        "MlpPolicy",
        train_envs,
        verbose=0,
        tensorboard_log=log_dir,
        policy_kwargs={"activation_fn": torch.nn.ReLU, "net_arch": [64, 64]},
    )
    model.learn(
        total_timesteps=n_learning_steps, callback=eval_callback, progress_bar=True
    )
