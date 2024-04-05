"""Script for running reinforcement learning on the drone forest."""

import matplotlib.pyplot as plt

from stable_baselines3 import PPO

# from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.callbacks import EvalCallback
from stable_baselines3.common.utils import set_random_seed
from stable_baselines3.common.vec_env import SubprocVecEnv

from drone_forest.gym_wrapper import DroneForestEnv


def make_env(rank: int, seed: int = 0) -> DroneForestEnv:
    """Make the drone forest environment."""

    def _init() -> DroneForestEnv:
        env = DroneForestEnv(
            dt=0.1,
            xlim=(-10, 10),
            ylim=(-1, 25),
            n_trees=200,
            max_tree_radius=1.0,
            n_lidar_beams=36,
            max_lidar_range=3.0,
            seed=seed + rank,
        )
        env.reset(seed=seed + rank)
        return env

    set_random_seed(seed)
    return _init


if __name__ == "__main__":
    # Set the parameters for the simulation
    n_envs = 8
    n_learning_steps = 25_000
    n_eval_steps = 1_000

    # Create the environments
    vec_env = SubprocVecEnv([make_env(i) for i in range(n_envs)])

    # Create the reinforcement learning model
    model = PPO("MlpPolicy", vec_env, verbose=1)
    model.learn(total_timesteps=n_learning_steps)

    # Run the evaluation
    plt.ion()
    obs = vec_env.reset()
    for _ in range(n_eval_steps):
        action, _ = model.predict(obs)
        obs, rewards, dones, info = vec_env.step(action)
        vec_env.render()

    # Close the plot
    plt.ioff()
