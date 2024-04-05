"""Unit tests for the forest class."""

import unittest

import matplotlib.pyplot as plt
import numpy as np

from drone_forest.forest import Forest


class TestForest(unittest.TestCase):
    """Test the forest class."""

    def test_forest_init(self):
        """Test the initialization of the forest object."""
        forest = Forest((-10, 10), (-10, 10), 5, 1)
        self.assertLessEqual(len(forest.trees), 5)

        forest = Forest((-10, 10), (-10, 10), 10, 0.5)
        self.assertLessEqual(len(forest.trees), 10)

    def test_forest_init_negative_radius(self):
        """Test the initialization of the forest object with a negative radius."""
        with self.assertRaises(AssertionError):
            Forest((-10, 10), (-10, 10), 5, -1)

    def test_forest_init_zero_trees(self):
        """Test the initialization of the forest object with zero trees."""
        with self.assertRaises(AssertionError):
            Forest((-10, 10), (-10, 10), 0, 1)

    def test_forest_init_negative_spare_distance(self):
        """Test the initialization of the forest object with a negative"""
        """ minimum spare distance."""
        with self.assertRaises(AssertionError):
            Forest((-10, 10), (-10, 10), 5, 1, -1)

    def test_forest_init_negative_max_spawn_attempts(self):
        """Test the initialization of the forest object with a negative"""
        """ max spawn attempts."""
        with self.assertRaises(AssertionError):
            Forest((-10, 10), (-10, 10), 5, 1, 0.2, -1)

    def test_forest_init_invalid_xlim(self):
        """Test the initialization of the forest object with invalid x-axis limits."""
        with self.assertRaises(AssertionError):
            Forest((10, -10), (-10, 10), 5, 1)

    def test_forest_init_invalid_ylim(self):
        """Test the initialization of the forest object with invalid y-axis limits."""
        with self.assertRaises(AssertionError):
            Forest((-10, 10), (10, -10), 5, 1)

    def test_forest_min_spare_distance(self):
        """Test the minimum spare distance of the forest object."""
        min_spare_distance = 0.5

        # Case 1: Spare forest
        forest = Forest((-10, 10), (-10, 10), 5, 1, min_spare_distance)
        self.assertLessEqual(len(forest.trees), 5)
        self.assertGreaterEqual(len(forest.trees), 1)
        for tree in forest.trees:
            for other in forest.trees:
                if tree != other:
                    self.assertGreaterEqual(
                        np.linalg.norm(tree.circle.center - other.circle.center),
                        min_spare_distance + tree.circle.radius + other.circle.radius,
                    )

        # Case 2: Dense forest
        forest = Forest((-10, 10), (-10, 10), 5000, 1, min_spare_distance)
        self.assertLessEqual(len(forest.trees), 5000)
        self.assertGreaterEqual(len(forest.trees), 1)
        for tree in forest.trees:
            for other in forest.trees:
                if tree != other:
                    self.assertGreaterEqual(
                        np.linalg.norm(tree.circle.center - other.circle.center),
                        min_spare_distance + tree.circle.radius + other.circle.radius,
                    )

    def test_forest_draw(self):
        """Test the draw method of the forest object."""
        ax = plt.gca()

        forest = Forest((-10, 10), (-10, 10), 5, 1, seed=13)
        forest.draw(ax)

        ax.clear()
        forest = Forest((-10, 10), (-10, 10), 10, 0.5, seed=13)
        forest.draw(ax)

        # ax.set_xlim(-10, 10)
        # ax.set_ylim(-10, 10)
        # ax.set_aspect("equal")
        # plt.show()

    def test_forest_draw_no_ax(self):
        """Test the draw method of the forest object without an axis."""
        forest = Forest((-10, 10), (-10, 10), 5, 1)
        with self.assertRaises(AssertionError):
            forest.draw(None)


if __name__ == "__main__":
    unittest.main()
