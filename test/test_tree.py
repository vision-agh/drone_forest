"""Unit tests for the tree class."""

import unittest

import matplotlib.pyplot as plt

from drone_forest.geometric_objects import Point
from drone_forest.tree import Tree


class TestTree(unittest.TestCase):
    """Test the tree class."""

    def test_tree_init(self):
        """Test the initialization of the tree object."""
        tree = Tree(Point(0, 0), 1)
        self.assertEqual(tree.circle.center, Point(0, 0))
        self.assertEqual(tree.circle.radius, 1)

        tree = Tree(Point(3, 4), 5)
        self.assertEqual(tree.circle.center, Point(3, 4))
        self.assertEqual(tree.circle.radius, 5)

        tree = Tree(Point(0, 0), 0)
        self.assertEqual(tree.circle.center, Point(0, 0))
        self.assertEqual(tree.circle.radius, 0)

    def test_tree_init_negative_radius(self):
        """Test the initialization of the tree object with a negative radius."""
        with self.assertRaises(AssertionError):
            Tree(Point(0, 0), -1)

    def test_tree_repr(self):
        """Test the string representation of the tree object."""
        tree = Tree(Point(0, 0), 1)
        self.assertEqual(repr(tree), "Tree(position=Point(x=0, y=0), radius=1)")

        tree = Tree(Point(3, 4), 5)
        self.assertEqual(repr(tree), "Tree(position=Point(x=3, y=4), radius=5)")

        tree = Tree(Point(0, 0), 0)
        self.assertEqual(repr(tree), "Tree(position=Point(x=0, y=0), radius=0)")

    def test_tree_draw(self):
        """Test the draw method of the tree object."""
        ax = plt.gca()

        tree = Tree(Point(0, 0), 0.5)
        tree.draw(ax)

        tree = Tree(Point(3, 4), 1)
        tree.draw(ax)

        tree = Tree(Point(-2, -3), 0.25)
        tree.draw(ax)

    def test_tree_draw_no_ax(self):
        """Test the draw method of the tree object without an axis."""
        tree = Tree(Point(0, 0), 0.5)
        with self.assertRaises(AssertionError):
            tree.draw(None)

    def test_tree_get_position(self):
        """Test the get_position method of the tree object."""
        tree = Tree(Point(0, 0), 1)
        self.assertEqual(tree.get_position(), Point(0, 0))

        tree = Tree(Point(3, 4), 5)
        self.assertEqual(tree.get_position(), Point(3, 4))

        tree = Tree(Point(-2, -3), 0.25)
        self.assertEqual(tree.get_position(), Point(-2, -3))

    def test_tree_get_radius(self):
        """Test the get_radius method of the tree object."""
        tree = Tree(Point(0, 0), 1)
        self.assertEqual(tree.get_radius(), 1)

        tree = Tree(Point(3, 4), 5)
        self.assertEqual(tree.get_radius(), 5)

        tree = Tree(Point(-2, -3), 0.25)
        self.assertEqual(tree.get_radius(), 0.25)


if __name__ == "__main__":
    unittest.main()
