"""Script for manual control of the drone."""

import random

from scripts.gym_wrapper import DroneForestEnv

# Set the parameters for the simulation
actions = [[0.0, 1.0], [-1.0, 0.0], [0.0, -1.0], [1.0, 0.0]]
dt = 0.1
x_lim = (-10.0, 10.0)
y_lim = (-2.0, 23.0)
y_static_limit = 18.0
n_trees = 100
tree_radius_lim = (0.1, 0.75)
n_lidar_beams = 72
lidar_range = 3.0
min_tree_spare_distance = 0.75
max_spawn_attempts = 100
max_speed = 1.0
max_acceleration = 0.6
drone_width = 0.2
drone_height = 0.4

# Create the simulation
env = DroneForestEnv(
    actions=actions,
    dt=dt,
    x_lim=x_lim,
    y_lim=y_lim,
    y_static_limit=y_static_limit,
    goal_y=y_lim[1] - 2.0,
    n_trees=n_trees,
    tree_radius_lim=tree_radius_lim,
    n_lidar_beams=n_lidar_beams,
    lidar_range=lidar_range,
    min_tree_spare_distance=min_tree_spare_distance,
    max_spawn_attempts=max_spawn_attempts,
    max_speed=max_speed,
    max_acceleration=max_acceleration,
    drone_width_m=drone_width,
    drone_height_m=drone_height,
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
        obs, reward, terminated, truncated, _ = env.step(action_id)
        if terminated or truncated:
            break
    choice = input("Do you want to continue? [y/n]: ")
    if choice.lower() != "y":
        env.close()
        break
