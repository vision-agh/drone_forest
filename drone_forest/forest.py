"""Script for the forest class."""

import numpy as np
from typing import List, Tuple

from drone_forest.tree import Tree
from drone_forest.geometric_objects import Circle, Point
from drone_forest.geometric_utils import distance


class Forest:
    """A forest object in the drone forest.

    Attributes:
        trees (List[Tree]): The trees in the forest.
    """

    def __init__(
        self,
        xlim: Tuple[float, float],
        ylim: Tuple[float, float],
        n_trees: int,
        max_radius: float,
        exclusion_zones: List[Circle] = [],
        min_spare_distance: float = 0.2,
        max_spawn_attempts: int = 50,
    ):
        """Initialize the forest object.

        Args:
            trees (List[Tree]): The trees in the forest.
        """
        assert max_radius >= 0, "The maximum radius of the trees must be non-negative."
        assert n_trees > 0, "The number of trees must be positive."
        assert (
            min_spare_distance >= 0
        ), "The minimum spare distance must be non-negative."
        assert max_spawn_attempts > 0, "The maximum spawn attempts must be positive."
        assert xlim[0] < xlim[1], "The x-axis limits must be in increasing order."
        assert ylim[0] < ylim[1], "The y-axis limits must be in increasing order."

        self.trees: List[Tree] = []
        self.xlim = xlim
        self.ylim = ylim

        # Generate random trees
        for _ in range(n_trees):
            # Try to spawn a tree
            for _ in range(max_spawn_attempts):
                x_pos = np.random.uniform(xlim[0], xlim[1])
                y_pos = np.random.uniform(ylim[0], ylim[1])
                radius = np.random.uniform(
                    0.05,
                    min(
                        max_radius,
                        xlim[1] - x_pos,
                        x_pos - xlim[0],
                        ylim[1] - y_pos,
                        y_pos - ylim[0],
                    ),
                )

                # Check if the tree is too close to another tree
                if any(
                    distance(Point(x_pos, y_pos), tree.get_position())
                    < radius + tree.get_radius() + min_spare_distance
                    for tree in self.trees
                ):
                    continue

                # Check if the tree is not in the exclusion zone
                if any(
                    distance(Point(x_pos, y_pos), circle.center)
                    < radius + circle.radius
                    for circle in exclusion_zones
                ):
                    continue

                self.trees.append(Tree(Point(x_pos, y_pos), radius))
                break

    def draw(self, ax):
        """Draw the forest on the given axis.

        Args:
            ax (matplotlib.axes.Axes): The axis to draw the forest on.
        """
        assert ax is not None, "The axis must be provided."

        for tree in self.trees:
            tree.draw(ax)
