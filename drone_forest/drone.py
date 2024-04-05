"""Script for the drone class."""

import cv2
import numpy as np

from drone_forest.geometric_objects import Point
from drone_forest.lidar import Lidar
from drone_forest.render_utils import transform_coords_to_render, IMAGE_HEIGHT


class Drone:
    """A drone object in the drone forest.

    Attributes:
        position (Point): The position of the drone.
        velocity (Point): The velocity of the drone.
        lidar (Lidar): The lidar attached to the drone.
    """

    def __init__(self, position: Point, lidar: Lidar):
        """Initialize the drone object.

        Args:
            position (Point): The position of the drone.
            lidar (Lidar): The lidar attached to the drone.
        """
        self.position: Point = position
        self.velocity: Point = Point(0, 0)
        self.lidar: Lidar = lidar
        self.update_lidar_position()

    def draw(self, img: np.ndarray, t_vec: Point, m2px: float):
        """Draw the drone on the given axis.

        Args:
            ax (matplotlib.axes.Axes): The axis to draw the drone on.
        """
        assert img is not None, "The axis must be provided."

        # Draw the lidar attached to the drone
        self.lidar.draw(img, t_vec, m2px)

        # Draw the drone as a small rectangle
        coord_bottom_left = transform_coords_to_render(
            Point(self.position.x - 0.05, self.position.y - 0.1), t_vec
        )
        coord_top_right = transform_coords_to_render(
            Point(self.position.x + 0.05, self.position.y + 0.1), t_vec
        )
        cv2.rectangle(
            img,
            (
                int(coord_bottom_left.x * m2px),
                int(coord_bottom_left.y * m2px) + IMAGE_HEIGHT,
            ),
            (
                int(coord_top_right.x * m2px),
                int(coord_top_right.y * m2px) + IMAGE_HEIGHT,
            ),
            (0, 0, 0),
            -1,
        )

    def move(self, dt: float, v: Point):
        """Move the drone by a given velocity.

        Args:
            dt (float): The time step for the movement.
            v (Point): The velocity of the drone.
        """
        self.velocity = v
        self.position.x += self.velocity.x * dt
        self.position.y += self.velocity.y * dt
        self.update_lidar_position()

    def update_lidar_position(self):
        """Update the position of the lidar attached to the drone."""
        self.lidar.position = self.position
