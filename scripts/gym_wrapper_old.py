"""Gymnasium wrapper for the drone forest environment."""

import cv2
import gymnasium as gym
import numpy as np
from typing import Tuple, Union

from drone_forest.geometric_objects import Point
from drone_forest.simulation import Simulation
from drone_forest.render_utils import IMAGE_HEIGHT

N_ACTIONS = 4
MAX_SIM_TIME_S = 120.0

TREE_SAFE_DISTANCE = 0.15
TREE_COLLISION_DISTANCE = 0.2
PENALTY_CLOSE_TO_TREE = -0.25
PENALTY_TREE_COLLISION = -1.5
PENALTY_FAR_FROM_CENTER_LINE_COEFF = -0.1
PENALTY_WRONG_DIRECTION_COEFF = -3.0
REWARD_GOOD_DIRECTION_COEFF = 0.8


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
        render_mode: str = "human",
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

        super(DroneForestEnv, self).__init__()
        self.render_mode = render_mode

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
            low=0.0, high=max_lidar_range, shape=(n_lidar_beams,), dtype=np.float64
        )

        # Rendering
        if self.render_mode == "human":
            self.m2px = IMAGE_HEIGHT / (ylim[1] - ylim[0])
            render_width = int((xlim[1] - xlim[0]) * self.m2px)
            # cv2.namedWindow("Drone Forest", cv2.WINDOW_NORMAL)
            # cv2.resizeWindow("Drone Forest", render_width, RENDER_HEIGHT)
            self.img = np.zeros((IMAGE_HEIGHT, render_width, 3), dtype=np.uint8)

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

        observation = np.array(self.simulation.step(control_value), dtype=np.float64)

        # Terminated, truncated and reward
        if (
            np.any(observation < TREE_COLLISION_DISTANCE)
            or self.simulation.drone.position.y <= self.ylim[0]
            or self.simulation.drone.position.y >= self.ylim[1]
            or self.simulation.drone.position.x <= self.xlim[0]
            or self.simulation.drone.position.x >= self.xlim[1]
        ):
            # Drone collided with a tree or went out of bounds
            reward = PENALTY_TREE_COLLISION
            terminated = True
        elif self.simulation.drone.position.x >= self.xlim[1] - 2.0:
            # Drone reached the end of the forest
            terminated = True
        else:
            reward = 0
            terminated = False
        truncated = self.simulation.sim_time >= MAX_SIM_TIME_S

        # Reward shaping
        if not terminated:
            # Center line reward
            reward += PENALTY_FAR_FROM_CENTER_LINE_COEFF * abs(
                self.simulation.drone.position.x
            )

            # Moving direction reward
            reward += (
                REWARD_GOOD_DIRECTION_COEFF
                if self.simulation.drone.velocity.y > 0
                else PENALTY_WRONG_DIRECTION_COEFF
            )

            # Penalty for being close to trees
            for tree in self.simulation.forest.trees:
                distance_to_tree = np.linalg.norm(
                    self.simulation.drone.position - tree.circle.center
                )
                if distance_to_tree < TREE_SAFE_DISTANCE:
                    reward += PENALTY_CLOSE_TO_TREE

        # Info
        info = {}

        return observation, reward, terminated, truncated, info

    def reset(self, seed: Union[None, int] = None, options=None):
        """Reset the environment."""
        super().reset(seed=seed, options=options)

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

        return np.array(self.simulation.step(Point(0, 0)), dtype=np.float64), {}

    def render(self):
        """Render the environment."""
        if self.render_mode == "human":
            self.img[:] = (0, 255, 0)
            self.simulation.draw(self.img, self.m2px)
            cv2.imshow("Drone Forest", self.img)
            cv2.waitKey(1)

    def close(self):
        """Close the environment."""
        pass