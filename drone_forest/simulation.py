"""Script for the simulation class."""

from drone_forest.geometric_objects import Circle, Point
from drone_forest.forest import Forest
from drone_forest.lidar import Lidar
from drone_forest.drone import Drone

import numpy as np
from typing import List, Tuple


class Simulation:
    """A simulation object in the drone forest.

    Attributes:
        forest (Forest): The forest in the simulation.
        lidar (Lidar): The lidar in the simulation.
    """

    def __init__(
        self,
        dt: float,
        xlim: Tuple[float, float],
        ylim: Tuple[float, float],
        n_trees: int,
        max_tree_radius: float,
        n_lidar_beams: int,
        max_lidar_range: float,
        min_spare_distance: float = 0.2,
        max_spawn_attempts: int = 50,
    ):
        """Initialize the simulation object.

        Args:
            xlim (Tuple[float, float]): The x-axis limits of the simulation.
            ylim (Tuple[float, float]): The y-axis limits of the simulation.
            n_trees (int): The number of trees in the forest.
            max_radius (float): The maximum radius of the trees in the forest.
            n_beams (int): The number of beams emitted by the lidar.
            max_range (float): The maximum range of the lidar.
            min_spare_distance (float): The minimum spare distance between trees.
            max_spawn_attempts (int): The maximum number of spawn attempts for trees.
            seed (int): The seed for reproducibility.
        """
        assert xlim[0] < 0 < xlim[1], "The x-axis limits must contain zero."
        assert ylim[0] < 0 < ylim[1], "The y-axis limits must contain zero."

        self.dt = dt
        self.forest = Forest(
            xlim,
            ylim,
            n_trees,
            max_tree_radius,
            [Circle(Point(0, 0), 1)],
            min_spare_distance,
            max_spawn_attempts,
        )
        self.drone = Drone(
            Point(0, 0), Lidar(Point(0, 0), max_lidar_range, n_lidar_beams)
        )
        self.sim_time = 0.0
        self.t_vec = Point(-self.forest.xlim[0], -self.forest.ylim[0])

    def draw(self, img: np.ndarray, m2px: float):
        """Draw the simulation."""
        # Draw the trees in the forest
        self.forest.draw(img, self.t_vec, m2px)

        # Draw the lidar
        self.drone.draw(img, self.t_vec, m2px)

    def step(self, dvec: Point) -> List[float]:
        """Take a step in the simulation."""
        # Move the drone
        self.drone.move(self.dt, dvec)

        # Update the simulation time
        self.sim_time += self.dt

        # Update the lidar scan
        return self.drone.lidar.scan([tree.circle for tree in self.forest.trees])
