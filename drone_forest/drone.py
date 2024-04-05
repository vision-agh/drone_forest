"""Script for the drone class."""

import matplotlib.pyplot as plt

from drone_forest.geometric_objects import Point
from drone_forest.lidar import Lidar


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

    def draw(self, ax: plt.Axes):
        """Draw the drone on the given axis.

        Args:
            ax (matplotlib.axes.Axes): The axis to draw the drone on.
        """
        assert ax is not None, "The axis must be provided."

        # Draw the lidar attached to the drone
        self.lidar.draw(ax)

        # Draw the drone as a small rectangle
        drone_rectangle = plt.Rectangle(
            (self.position.x - 0.05, self.position.y - 0.1),
            0.1,
            0.2,
            color="black",
            fill=True,
        )
        ax.add_patch(drone_rectangle)

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
