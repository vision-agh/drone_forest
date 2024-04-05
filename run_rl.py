"""Script for running reinforcement learning on the drone forest."""

import numpy as np
import matplotlib.pyplot as plt

# from stable_baselines3 import A2C

from drone_forest.gym_wrapper import DroneForestEnv

# Set the seed for reproducibility
np.random.seed(0)

# Set the parameters for the simulation
dt = 0.1
xlim = (-10, 10)
ylim = (-1, 25)
n_trees = 200
max_tree_radius = 1.0
n_lidar_beams = 36
max_lidar_range = 3.0

# Create the gym environment
env = DroneForestEnv(
    dt,
    xlim,
    ylim,
    n_trees,
    max_tree_radius,
    n_lidar_beams,
    max_lidar_range,
    seed=0,
)

# Set up the plot
plt.ion()
ax: plt.Axes
figure, ax = plt.subplots()

# Run the reinforcement learning loop
n_episodes = 3
obs = env.reset(0)
for episode in range(n_episodes):
    while True:
        action = env.action_space.sample()
        obs, reward, terminated, truncated, info = env.step(action)

        # Show the simulation window
        ax.clear()
        ax.fill_between(
            [xlim[0], xlim[1]],
            [ylim[0], ylim[0]],
            [ylim[1], ylim[1]],
            color="green",
            alpha=0.7,
        )
        env.render(ax)
        ax.set_aspect("equal")
        ax.set_xlim(xlim)
        ax.set_ylim(ylim)
        figure.canvas.draw()
        figure.canvas.flush_events()

        # Check if the episode is done
        if terminated or truncated:
            if terminated:
                print(f"Episode {episode + 1} terminated with reward {reward}.")
            else:
                print(f"Episode {episode + 1} truncated with reward {reward}.")
            break
    obs = env.reset()

# Close the plot
plt.ioff()
