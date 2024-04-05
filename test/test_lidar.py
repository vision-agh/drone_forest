"""Unit tests for the lidar class."""

import unittest

import math
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
        # Case 1: The lidar is at the origin and has 180 beams (2 degrees apart). The
        # obstacles is placed in such a way that 9 beams hit the obstacle.
        # NOTE: The distance to the obstacle is calculated using the law of cosines
        d = 2.0
        r = 0.28
        lidar_range = 5
        n_beams = 180
        beam_ang = 2 * math.pi / n_beams
        lidar = Lidar(Point(0, 0), lidar_range, n_beams)
        obstacles = [
            Tree(Point(d, 0), r),
        ]
        distances_gt = [lidar_range] * n_beams
        distances_gt[0] = d - r
        for i in range(1, 5):
            distances_gt[i] = d * (
                math.cos(i * beam_ang)
                - math.sqrt(math.cos(i * beam_ang) ** 2 - 1 + (r / d) ** 2)
            )
            distances_gt[-i] = distances_gt[i]
        distances = lidar.scan([obstacle.circle for obstacle in obstacles])
        for i in range(5):
            self.assertAlmostEqual(distances[i], distances_gt[i], places=2)
            self.assertAlmostEqual(distances[-i], distances_gt[-i], places=2)
        self.assertEqual(distances[5:-4], [5] * (180 - 9))

        # Case 2: The lidar is at the origin and has 90 beams (4 degrees apart). The
        # obstacles is placed in such a way that 20 beams hit the obstacle.
        # NOTE: The distance to the obstacle is calculated using the law of cosines
        d = 1.5
        r = 0.97
        lidar_range = 5
        n_beams = 90
        beam_ang = 2 * math.pi / n_beams
        lidar = Lidar(Point(0, 0), lidar_range, n_beams)
        obstacles = [
            Tree(Point(0, d), r),
        ]
        distances_gt = [lidar_range] * n_beams
        for i in range(10):
            cos_ang = math.pi / 2 - (22 - i) * beam_ang
            distances_gt[22 - i] = d * (
                math.cos(cos_ang) - math.sqrt(math.cos(cos_ang) ** 2 - 1 + (r / d) ** 2)
            )
            distances_gt[23 + i] = distances_gt[22 - i]
        distances = lidar.scan([obstacle.circle for obstacle in obstacles])
        for i in range(10):
            self.assertAlmostEqual(distances[22 - i], distances_gt[22 - i], places=1)
            self.assertAlmostEqual(distances[23 + i], distances_gt[23 + i], places=1)
        self.assertEqual(distances[:13], [5] * 13)
        self.assertEqual(distances[33:], [5] * 57)

        # ax = plt.gca()
        # for obstacle in obstacles:
        #     obstacle.draw(ax)
        # lidar.draw(ax)
        # ax.set_xlim(-20, 20)
        # ax.set_ylim(-20, 20)
        # ax.set_aspect("equal")
        # plt.show()

    def test_lidar_scan_no_obstacles(self):
        """Test the scan method of the lidar object without obstacles."""
        lidar = Lidar(Point(0, 0), 1, 1)
        self.assertEqual(lidar.scan([]), [1])

        lidar = Lidar(Point(3, 4), 5, 90)
        self.assertEqual(lidar.scan([]), [5] * 90)

        lidar = Lidar(Point(0, 0), 2, 180)
        self.assertEqual(lidar.scan([]), [2] * 180)

    def test_lidar_obstacles_hiding(self):
        """Test the scan method of the lidar object when obstacles are hidden by other \
            obstacles."""
        # Case 1: The lidar is at the origin and has 180 beams (2 degrees apart). The
        # obstacles are placed in such a way that 9 beams hit the first obstacle and 9
        # beams hit the second obstacle.
        d = 2.0
        r = 0.28
        lidar_range = 5
        n_beams = 180
        beam_ang = 2 * math.pi / n_beams
        lidar = Lidar(Point(0, 0), lidar_range, n_beams)
        obstacles = [
            Tree(Point(d, 0), r),
            Tree(Point(d + 2 * r, 0), r),
        ]
        distances_gt = [lidar_range] * n_beams
        distances_gt[0] = d - r
        for i in range(1, 5):
            distances_gt[i] = d * (
                math.cos(i * beam_ang)
                - math.sqrt(math.cos(i * beam_ang) ** 2 - 1 + (r / d) ** 2)
            )
            distances_gt[-i] = distances_gt[i]
        distances = lidar.scan([obstacle.circle for obstacle in obstacles])
        for i in range(5):
            self.assertAlmostEqual(distances[i], distances_gt[i], places=2)
            self.assertAlmostEqual(distances[-i], distances_gt[-i], places=2)
        self.assertEqual(distances[5:-4], [5] * (180 - 9))

        # Case 2: The lidar is at the origin and has 90 beams (4 degrees apart). The
        # obstacles are placed in such a way that 20 beams hit the first obstacle and 20
        # beams hit the second obstacle.
        d = 1.5
        r = 0.97
        lidar_range = 5
        n_beams = 90
        beam_ang = 2 * math.pi / n_beams
        lidar = Lidar(Point(0, 0), lidar_range, n_beams)
        obstacles = [
            Tree(Point(0, d), r),
            Tree(Point(0, d + 2 * r), r / 2),
        ]
        distances_gt = [lidar_range] * n_beams
        for i in range(10):
            cos_ang = math.pi / 2 - (22 - i) * beam_ang
            distances_gt[22 - i] = d * (
                math.cos(cos_ang) - math.sqrt(math.cos(cos_ang) ** 2 - 1 + (r / d) ** 2)
            )
            distances_gt[23 + i] = distances_gt[22 - i]
        distances = lidar.scan([obstacle.circle for obstacle in obstacles])
        for i in range(10):
            self.assertAlmostEqual(distances[22 - i], distances_gt[22 - i], places=1)
            self.assertAlmostEqual(distances[23 + i], distances_gt[23 + i], places=1)
        self.assertEqual(distances[:13], [5] * 13)
        self.assertEqual(distances[33:], [5] * 57)

        # ax = plt.gca()
        # for obstacle in obstacles:
        #     obstacle.draw(ax)
        # lidar.draw(ax)
        # ax.set_xlim(-20, 20)
        # ax.set_ylim(-20, 20)
        # ax.set_aspect("equal")
        # plt.show()


if __name__ == "__main__":
    unittest.main()
