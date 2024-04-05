"""Script for manual control of the drone."""

import random

from drone_forest.gym_wrapper import DroneForestEnv

# Set the parameters for the simulation
dt = 0.1
xlim = (-10, 10)
ylim = (-1, 25)
n_trees = 200
max_tree_radius = 1.0
n_lidar_beams = 36
max_lidar_range = 3.0

# Create the simulation
env = DroneForestEnv(
    dt=dt,
    xlim=xlim,
    ylim=ylim,
    n_trees=n_trees,
    max_tree_radius=max_tree_radius,
    n_lidar_beams=n_lidar_beams,
    max_lidar_range=max_lidar_range,
)
env.reset()

while True:
    for _ in range(500):
        env.render()

        # Get the user input
        action = random.choice(["w", "a", "s", "d"])

        # Perform the action
        if action == "w":
            action_id = 0
        elif action == "a":
            action_id = 1
        elif action == "s":
            action_id = 2
        elif action == "d":
            action_id = 3
        else:
            print("Invalid action.")
            continue

        # Perform simulation step
        env.step(action_id)
    choice = input("Do you want to continue? [y/n]: ")
    if choice.lower() != "y":
        break
