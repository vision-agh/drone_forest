"""Script for manual control of the drone."""

import matplotlib.pyplot as plt

from drone_forest.geometric_objects import Point
from drone_forest.simulation import Simulation

# Set the parameters for the simulation
dt = 0.1
xlim = (-10, 10)
ylim = (-1, 25)
n_trees = 200
max_tree_radius = 1.0
n_lidar_beams = 36
max_lidar_range = 3.0

# Create the simulation
simulation = Simulation(
    dt,
    xlim,
    ylim,
    n_trees,
    max_tree_radius,
    n_lidar_beams,
    max_lidar_range,
)

# Set up the plot
plt.ion()
ax: plt.Axes
figure, ax = plt.subplots()

# Manual control in the loop
simulation.step(Point(0, 0))
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
    control_action: Point = Point(0, 0)
    if action == "w":
        control_action = Point(0, 1)
    elif action == "a":
        control_action = Point(-1, 0)
    elif action == "s":
        control_action = Point(0, -1)
    elif action == "d":
        control_action = Point(1, 0)
    elif action == "q":
        break

    # Perform simulation step
    simulation.step(control_action)
