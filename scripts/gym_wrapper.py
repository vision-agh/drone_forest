"""Gymnasium wrapper for the drone forest environment."""

import os
import sys

script_dir = os.path.abspath(os.path.dirname(__file__))

# Add pybind11 wrapper for the drone forest environment
module_dir = os.path.abspath(os.path.join(script_dir, "../build/scripts"))

if not os.path.exists(module_dir):
    print(f"Cannot find the pybind11 wrapper at {module_dir}.")
    print("Please build the pybind11 wrapper first.")
    exit(1)

if module_dir not in sys.path:
    sys.path.append(module_dir)

from drone_forest import DroneForest

import cv2
import gymnasium as gym
import numpy as np
from typing import Dict, List, Tuple, Union

MAX_SIM_TIME_S = 120.0

TREE_SAFE_DISTANCE = 0.15
TREE_COLLISION_DISTANCE = 0.2
PENALTY_CLOSE_TO_TREE = -0.25
PENALTY_TREE_COLLISION = -1.5
PENALTY_FAR_FROM_CENTER_LINE_COEFF = -0.1
PENALTY_WRONG_DIRECTION_COEFF = -3.0
REWARD_GOOD_DIRECTION_COEFF = 0.8

X = 0
Y = 1


class DroneForestEnv(gym.Env):
    """Drone forest environment that follows the gym interface."""

    metadata = {"render.modes": ["human"]}

    def __init__(
        self,
        actions: List[List[float]],
        dt: float,
        x_lim: Tuple[float, float],
        y_lim: Tuple[float, float],
        n_trees: int,
        tree_radius_lim: Tuple[float, float],
        n_lidar_beams: int,
        lidar_range: float,
        min_tree_spare_distance: float,
        max_spawn_attempts: int,
        max_speed: float,
        max_acceleration: float,
        drone_width_m: float,
        drone_height_m: float,
        render_mode: str = "human",
    ):
        """Initialize the drone forest environment.

        Args:
            actions (List[List[float]]): The list of actions to take.
            dt (float): The simulation time step.
            x_lim (Tuple[float, float]): The x-axis limits of the simulation.
            y_lim (Tuple[float, float]): The y-axis limits of the simulation.
            n_trees (int): The number of trees in the forest.
            tree_radius_lim (Tuple[float, float]): The limits of the tree radii.
            n_lidar_beams (int): The number of beams emitted by the lidar.
            lidar_range (float): The maximum range of the lidar.
            min_tree_spare_distance (float): The minimum spare distance between trees.
            max_spawn_attempts (int): The maximum number of spawn attempts for trees.
            max_speed (float): The maximum speed of the drone.
            max_acceleration (float): The maximum acceleration of the drone.
            render_mode (str, optional): The rendering mode. Defaults to "human".
        """
        assert x_lim[0] < 0 < x_lim[1], "The x-axis limits must contain zero."
        assert y_lim[0] < 0 < y_lim[1], "The y-axis limits must contain zero."

        super(DroneForestEnv, self).__init__()
        self.render_mode = render_mode

        self.x_lim = x_lim
        self.y_lim = y_lim
        self.drone_prev_position = [0.0, 0.0]
        self.env = DroneForest(
            sim_step=dt,
            x_lim=x_lim,
            y_lim=y_lim,
            n_trees=n_trees,
            tree_min_radius=tree_radius_lim[0],
            tree_max_radius=tree_radius_lim[1],
            n_lidar_beams=n_lidar_beams,
            lidar_range=lidar_range,
            min_tree_spare_distance=min_tree_spare_distance,
            max_spawn_attempts=max_spawn_attempts,
            max_speed=max_speed,
            max_acceleration=max_acceleration,
            drone_width_m=drone_width_m,
            drone_height_m=drone_height_m,
        )

        self.action_vec = actions
        self.action_space = gym.spaces.Discrete(len(actions))
        self.observation_space = gym.spaces.Box(
            low=0.0, high=lidar_range, shape=(n_lidar_beams,), dtype=np.float64
        )

    def step(self, action: int) -> Tuple[np.ndarray, float, bool, bool, dict]:
        """Perform an action in the environment.

        Args:
            action (int): The action ID to perform.

        Returns:
            Tuple[np.ndarray, float, bool, bool, dict]: The observation, reward, \
            termination flag, truncation flag, and info dictionary.
        """
        assert self.action_space.contains(action), f"Invalid action {action}."

        # Perform the action
        self.env.step(self.action_vec[action])

        # Get the observation
        obs = np.array(self.env.get_lidar_distances(), dtype=np.float64)

        # Determine termination or truncation
        drone_position = self.env.get_drone_position()
        if (
            self.env.check_collision()
            or drone_position[X] <= self.x_lim[0]
            or drone_position[X] >= self.x_lim[1]
            or drone_position[Y] <= self.y_lim[0]
            or drone_position[Y] >= self.y_lim[1]
        ):
            # Drone crashed or went out of bounds
            reward = PENALTY_TREE_COLLISION
            terminated = True
        elif drone_position[Y] >= self.y_lim[1] - 2.0:
            # Drone reached the end
            reward = 1.0
            terminated = True
        else:
            # Drone is still flying
            reward = 0.0
            terminated = False
        truncated = self.env.get_time() >= MAX_SIM_TIME_S

        # Reward shaping
        if not terminated:
            # Penalize being too far from the center line
            reward += PENALTY_FAR_FROM_CENTER_LINE_COEFF * abs(drone_position[X])

            # Penalize being too close to a tree
            for dist in obs:
                if dist < TREE_SAFE_DISTANCE:
                    reward += PENALTY_CLOSE_TO_TREE

            # Reward moving in the right direction or penalize moving in the wrong
            # direction
            if drone_position[Y] > self.drone_prev_position[Y]:
                reward += REWARD_GOOD_DIRECTION_COEFF
            else:
                reward += PENALTY_WRONG_DIRECTION_COEFF

        # Info
        info = {}

        # Update the previous drone position
        self.drone_prev_position = drone_position

        return obs, reward, terminated, truncated, info

    def reset(
        self, seed: Union[None, int] = None, options=None
    ) -> Tuple[np.ndarray, Dict]:
        """Reset the environment.

        Args:
            seed (Union[None, int], optional): Seed for the randomness generator. \
            Defaults to None.
            options (_type_, optional): Optional options. Defaults to None.

        Returns:
            Tuple[np.ndarray, Dict]: The initial observation and info dictionary.
        """
        super().reset(seed=seed, options=options)

        # Reset the environment
        if seed is not None:
            self.env.reset_seed(seed)
        else:
            self.env.reset()

        # Get the initial observation
        obs = np.array(self.env.get_lidar_distances(), dtype=np.float64)

        # Info
        info = {}

        return obs, info

    def render(self):
        """Render the environment."""
        if self.render_mode == "human":
            self.env.render()
            img_data: List[int] = self.env.get_image()
            img = np.array(img_data, dtype=np.uint8).reshape(
                *self.env.get_image_size(), -1
            )
            cv2.imshow("Drone Forest", img)
            cv2.waitKey(1)
        else:
            raise NotImplementedError("Only human rendering is supported.")

    def close(self):
        """Close the environment."""
        cv2.destroyAllWindows()
