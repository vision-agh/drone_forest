"""Unit tests for the lidar class."""

import unittest

import matplotlib.pyplot as plt

from drone_forest.geometric_objects import Point
from drone_forest.lidar import Lidar
from drone_forest.tree import Tree


class TestLidar(unittest.TestCase):
    """Test the lidar class."""

    def test_lidar_init(self):
        """Test the initialization of the lidar object."""
        lidar = Lidar(Point(3, 4), 5, 90)
        self.assertEqual(lidar.position, Point(3, 4))
        self.assertEqual(lidar.max_range, 5)
        self.assertEqual(len(lidar.beam_angles), 90)

        lidar = Lidar(Point(0, 0), 2, 180)
        self.assertEqual(lidar.position, Point(0, 0))
        self.assertEqual(lidar.max_range, 2)
        self.assertEqual(len(lidar.beam_angles), 180)

    def test_lidar_init_negative_max_range(self):
        """Test the initialization of the lidar object with a negative max_range."""
        with self.assertRaises(AssertionError):
            Lidar(Point(0, 0), -1, 0)

    def test_lidar_init_invalid_number_of_beams(self):
        """Test the initialization of the lidar object with an invalid number of \
            beams."""
        with self.assertRaises(AssertionError):
            Lidar(Point(0, 0), 1, -1)

        with self.assertRaises(AssertionError):
            Lidar(Point(0, 0), 1, 0)

    def test_lidar_draw(self):
        """Test the draw method of the lidar object."""
        ax = plt.gca()

        lidar = Lidar(Point(3, 4), 5, 90)
        lidar.draw(ax)
        ax.clear()

        lidar = Lidar(Point(-2, -3), 2, 180)
        ax = plt.gca()
        lidar.draw(ax)

    def test_lidar_draw_no_ax(self):
        """Test the draw method of the lidar object without an axis."""
        lidar = Lidar(Point(0, 0), 1, 1)
        with self.assertRaises(AssertionError):
            lidar.draw(None)

    def test_lidar_scan(self):
        """Test the scan method of the lidar object."""
        lidar = Lidar(Point(3, 4), 5, 90)
        obstacles = [
            Tree(Point(0, 0), 0.5),
            Tree(Point(10, 10), 1),
        ]
        distances = lidar.scan([obstacle.circle for obstacle in obstacles])
        # TODO: Test the distances
        ax = plt.gca()
        for obstacle in obstacles:
            obstacle.draw(ax)
        lidar.draw(ax)
        ax.set_xlim(-20, 20)
        ax.set_ylim(-20, 20)
        ax.set_aspect("equal")
        plt.show()

    def test_lidar_scan_no_obstacles(self):
        """Test the scan method of the lidar object without obstacles."""
        lidar = Lidar(Point(0, 0), 1, 1)
        self.assertEqual(lidar.scan([]), [1])

        lidar = Lidar(Point(3, 4), 5, 90)
        self.assertEqual(lidar.scan([]), [5] * 90)

        lidar = Lidar(Point(0, 0), 2, 180)
        self.assertEqual(lidar.scan([]), [2] * 180)


if __name__ == "__main__":
    unittest.main()
