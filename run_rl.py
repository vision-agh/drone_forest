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

import scripts.json_utils as jutils


def make_env(rank: int, seed: int = 0, config_dict: Dict = {}) -> DroneForestEnv:
    """Make the drone forest environment."""

    def _init() -> DroneForestEnv:
        env = DroneForestEnv(
            actions=config_dict["actions"],
            dt=config_dict["sim_step"],
            x_lim=(config_dict["x_lim"]["min"], config_dict["x_lim"]["max"]),
            y_lim=(config_dict["y_lim"]["min"], config_dict["y_lim"]["max"]),
            y_static_limit=config_dict["y_static_limit"],
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
    # Read environment configuration
    config_dict = jutils.read_env_config("./env_config.json")
    if config_dict is None:
        raise ValueError("The environment configuration is invalid.")

    # Check gym wrapper
    check_env(make_env(0, config_dict=config_dict)())

    # Read the RL configuration
    rl_dict = jutils.read_rl_config("./rl_config.json")
    if rl_dict is None:
        raise ValueError("The RL configuration is invalid.")

    # Create log dir where evaluation results will be saved
    exp_dir = str(int(time.time()))
    log_dir = os.path.join(f"./logs_{rl_dict['algorithm']}", exp_dir)
    os.makedirs(log_dir, exist_ok=True)

    # Preserve the environment configuration
    with open(os.path.join(log_dir, "env_config.json"), "w") as config_file:
        json.dump(config_dict, config_file)

    # Preserve the RL configuration
    with open(os.path.join(log_dir, "rl_config.json"), "w") as rl_file:
        json.dump(rl_dict, rl_file)

    # Create the environments
    train_envs = SubprocVecEnv(
        [
            make_env(i, config_dict=config_dict)
            for i in range(rl_dict["nb_training_envs"])
        ]
    )
    eval_envs = SubprocVecEnv(
        [make_env(i, config_dict=config_dict) for i in range(rl_dict["nb_eval_envs"])]
    )

    eval_callback = EvalCallback(
        eval_envs,
        best_model_save_path=log_dir,
        log_path=log_dir,
        eval_freq=max(rl_dict["nb_eval_every"] // rl_dict["nb_training_envs"], 1),
        n_eval_episodes=1,
        deterministic=True,
        render=False,
    )

    # Create the reinforcement learning model
    if rl_dict["activation_fn"] == "sigmoid":
        activation_fn = torch.nn.Sigmoid
    elif rl_dict["activation_fn"] == "tanh":
        activation_fn = torch.nn.Tanh
    else:
        activation_fn = torch.nn.ReLU

    net_arch = [rl_dict["nb_neurons"]] * rl_dict["nb_layers"]

    model = PPO(
        "MlpPolicy",
        train_envs,
        verbose=0,
        tensorboard_log=log_dir,
        policy_kwargs={"activation_fn": activation_fn, "net_arch": net_arch},
    )
    model.learn(
        total_timesteps=rl_dict["nb_training_steps"],
        callback=eval_callback,
        progress_bar=True,
    )
