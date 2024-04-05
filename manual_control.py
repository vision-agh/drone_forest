"""Script for manual control of the drone."""

import matplotlib.pyplot as plt

from drone_forest.simulation import Simulation

# Set the parameters for the simulation
xlim = (-10, 10)
ylim = (-1, 25)
n_trees = 200
max_tree_radius = 1.0
n_lidar_beams = 36
max_lidar_range = 3.0

# Create the simulation
simulation = Simulation(
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

# Manual control in the loop
simulation.step()
while True:
    # Show the simulation window
    ax.clear()
    ax.fill_between(
        [xlim[0], xlim[1]],
        [ylim[0], ylim[0]],
        [ylim[1], ylim[1]],
        color="green",
        alpha=0.7,
    )
    simulation.draw(ax)
    ax.set_aspect("equal")
    ax.set_xlim(xlim)
    ax.set_ylim(ylim)
    figure.canvas.draw()
    figure.canvas.flush_events()

    # Get the user input
    action = input("Enter an action (w: up, a: left, s: down, d: right, q: quit): ")

    # Perform the action
    if action == "w":
        simulation.lidar.position.y += 0.1
    elif action == "a":
        simulation.lidar.position.x -= 0.1
    elif action == "s":
        simulation.lidar.position.y -= 0.1
    elif action == "d":
        simulation.lidar.position.x += 0.1
    elif action == "q":
        break

    # Perform simulation step
    simulation.step()
