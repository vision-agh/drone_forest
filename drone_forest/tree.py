"""Script for the tree class."""

import matplotlib.pyplot as plt

from drone_forest.geometric_objects import Circle, Point


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

    def draw(self, ax):
        """Draw the tree on the given axis.

        Args:
            ax (matplotlib.axes.Axes): The axis to draw the tree on.
        """
        assert ax is not None, "The axis must be provided."

        circle = plt.Circle(
            (self.circle.center.x, self.circle.center.y),
            self.circle.radius,
            color="brown",
            fill=True,
        )
        ax.add_artist(circle)
