"""Gymnasium wrapper for the drone forest environment."""

import gymnasium as gym
import matplotlib.pyplot as plt
import numpy as np
from typing import Tuple, Union

from drone_forest.geometric_objects import Point
from drone_forest.simulation import Simulation

N_ACTIONS = 4
MAX_SIM_TIME_S = 120.0


class DroneForestEnv(gym.Env):
    """Drone forest environment that follows the gym interface."""

    metadata = {"render.modes": ["human"]}

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
        seed: int = 0,
    ):
        """Initialize the drone forest environment.

        Args:
            xlim (Tuple[float, float]): The x-axis limits of the simulation.
            ylim (Tuple[float, float]): The y-axis limits of the simulation.
            n_trees (int): The number of trees in the forest.
            max_tree_radius (float): The maximum radius of the trees in the forest.
            n_lidar_beams (int): The number of beams emitted by the lidar.
            max_lidar_range (float): The maximum range of the lidar.
            min_spare_distance (float): The minimum spare distance between trees.
            max_spawn_attempts (int): The maximum number of spawn attempts for trees.
            seed (int): The seed for reproducibility.
        """
        assert xlim[0] < 0 < xlim[1], "The x-axis limits must contain zero."
        assert ylim[0] < 0 < ylim[1], "The y-axis limits must contain zero."

        super().__init__()

        self.simulation: Simulation = None
        self.dt = dt
        self.xlim = xlim
        self.ylim = ylim
        self.n_trees = n_trees
        self.max_tree_radius = max_tree_radius
        self.n_lidar_beams = n_lidar_beams
        self.max_lidar_range = max_lidar_range
        self.min_spare_distance = min_spare_distance
        self.max_spawn_attempts = max_spawn_attempts
        self.seed = seed

        self.action_space = gym.spaces.Discrete(N_ACTIONS)
        self.observation_space = gym.spaces.Box(
            low=0.0, high=max_lidar_range, shape=(n_lidar_beams,), dtype=np.float32
        )

    def step(self, action: int):
        """Take a step in the environment."""
        assert self.action_space.contains(action), f"Invalid action: {action}"

        control_value: Point = Point(0, 0)
        if action == 0:
            control_value = Point(0, 1)
        elif action == 1:
            control_value = Point(-1, 0)
        elif action == 2:
            control_value = Point(0, -1)
        elif action == 3:
            control_value = Point(1, 0)

        observation = np.array(self.simulation.step(control_value))

        # Terminated, truncated and reward
        collision = (
            np.any(observation < 0.2)
            or self.simulation.drone.position.y <= self.ylim[0]
            or self.simulation.drone.position.y >= self.ylim[1]
            or self.simulation.drone.position.x <= self.xlim[0]
            or self.simulation.drone.position.x >= self.xlim[1]
        )
        success = self.simulation.drone.position.x >= self.xlim[1] - 2.0
        if collision:
            reward = -1
            terminated = True
        elif success:
            reward = 1
            terminated = True
        else:
            reward = 0
            terminated = False
        truncated = self.simulation.sim_time >= MAX_SIM_TIME_S

        # Info
        info = {}

        return observation, reward, terminated, truncated, info

    def reset(self, seed: Union[None, int] = None):
        """Reset the environment."""
        if seed is not None:
            np.random.seed(seed)

        self.simulation = Simulation(
            self.dt,
            self.xlim,
            self.ylim,
            self.n_trees,
            self.max_tree_radius,
            self.n_lidar_beams,
            self.max_lidar_range,
            self.min_spare_distance,
            self.max_spawn_attempts,
        )

        return np.array(self.simulation.step(Point(0, 0))), {}

    def render(self, ax: plt.Axes):
        """Render the environment."""
        self.simulation.draw(ax)

    def close(self):
        """Close the environment."""
        pass
