"""Script for the tree class."""

import cv2
import numpy as np

from drone_forest.geometric_objects import Circle, Point
from drone_forest.render_utils import transform_coords_to_render, IMAGE_HEIGHT


class Tree:
    """A tree object in the drone forest.

    Attributes:
        circle (Circle): The circle representing the tree.
    """

    def __init__(self, position: Point, radius: float):
        """Initialize the tree object.

        Args:
            position (Point): The position of the tree.
            radius (float): The radius of the tree.
        """
        assert radius >= 0, "The radius of the tree must be non-negative."

        self.circle = Circle(position, radius)

    def __repr__(self) -> str:
        """Return the string representation of the tree object.

        Returns:
            str: The string representation of the tree object.
        """
        return f"Tree(position={self.circle.center}, radius={self.circle.radius})"

    def draw(self, img: np.ndarray, t_vec: Point, m2px: float):
        """Draw the tree on the given axis.

        Args:
            ax (matplotlib.axes.Axes): The axis to draw the tree on.
        """
        assert img is not None, "The axis must be provided."

        coord_center = transform_coords_to_render(self.circle.center, t_vec)
        cv2.circle(
            img,
            (
                int(coord_center.x * m2px),
                int(coord_center.y * m2px) + IMAGE_HEIGHT,
            ),
            int(self.circle.radius * m2px),
            (42, 42, 166),
            -1,
        )

    def get_position(self) -> Point:
        """Get the position of the tree.

        Returns:
            Point: The position of the tree.
        """
        return self.circle.center

    def get_radius(self) -> float:
        """Get the radius of the tree.

        Returns:
            float: The radius of the tree.
        """
        return self.circle.radius
