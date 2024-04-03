"""Unit tests for the geometric functionalities."""

import unittest

from drone_forest.geometric_objects import Circle, Line, Point
from drone_forest.geometric_utils import (
    calculate_line_circle_intersection_points,
    check_line_circle_intersection,
    distance,
)


class TestGeometricUtils(unittest.TestCase):
    """Test the geometric utilities."""

    def test_distance(self):
        """Test the distance function."""
        p1 = Point(0, 0)
        p2 = Point(3, 4)
        self.assertEqual(distance(p1, p2), 5.0)

        p1 = Point(0, 0)
        p2 = Point(1, 1)
        self.assertEqual(distance(p1, p2), 2**0.5)

        p1 = Point(0, 0)
        p2 = Point(0, 0)
        self.assertEqual(distance(p1, p2), 0.0)

    def test_distance_symmetry(self):
        """Test the symmetry of the distance function."""
        p1 = Point(0, 0)
        p2 = Point(3, 4)
        self.assertEqual(distance(p1, p2), distance(p2, p1))

        p1 = Point(0, 0)
        p2 = Point(1, 1)
        self.assertEqual(distance(p1, p2), distance(p2, p1))

        p1 = Point(0, 0)
        p2 = Point(0, 0)
        self.assertEqual(distance(p1, p2), distance(p2, p1))

    def test_check_line_circle_intersection(self):
        """Test the check_line_circle_intersection function."""
        # Case 1: The line segment intersects the circle located at the origin
        line = Line(Point(0, 0), Point(3, 4))
        circle = Circle(Point(0, 0), 1)
        self.assertTrue(check_line_circle_intersection(line, circle))

        line = Line(Point(0, 0), Point(3, 4))
        circle = Circle(Point(0, 0), 0.5)
        self.assertTrue(check_line_circle_intersection(line, circle))

        line = Line(Point(0, 0), Point(3, 4))
        circle = Circle(Point(0, 0), 5)
        self.assertTrue(check_line_circle_intersection(line, circle))

        line = Line(Point(0, 0), Point(0, 0))
        circle = Circle(Point(0, 0), 1)
        self.assertTrue(check_line_circle_intersection(line, circle))

        line = Line(Point(0, 0), Point(0, 0))
        circle = Circle(Point(0, 0), 0.5)
        self.assertTrue(check_line_circle_intersection(line, circle))

        line = Line(Point(0, 0), Point(0, 0))
        circle = Circle(Point(0, 0), 0)
        self.assertTrue(check_line_circle_intersection(line, circle))

        # Case 2: The line segment intersects the circle located outside the origin
        line = Line(Point(0, 0), Point(3, 4))
        circle = Circle(Point(10, 10), 1)
        self.assertFalse(check_line_circle_intersection(line, circle))

        line = Line(Point(0, 0), Point(3, 4))
        circle = Circle(Point(10, 10), 0.5)
        self.assertFalse(check_line_circle_intersection(line, circle))

        line = Line(Point(0, 0), Point(3, 4))
        circle = Circle(Point(2, 2), 1)
        self.assertTrue(check_line_circle_intersection(line, circle))

        line = Line(Point(0, 0), Point(3, 4))
        circle = Circle(Point(2, 2), 0.5)
        self.assertTrue(check_line_circle_intersection(line, circle))

        line = Line(Point(0, 0), Point(3, 4))
        circle = Circle(Point(8, 4), 5)
        self.assertTrue(check_line_circle_intersection(line, circle))

        # Case 3: The line segment does not start at the origin
        line = Line(Point(1, 1), Point(3, 4))
        circle = Circle(Point(0, 0), 1)
        self.assertFalse(check_line_circle_intersection(line, circle))

        line = Line(Point(1, 1), Point(3, 4))
        circle = Circle(Point(0, 0), 0.5)
        self.assertFalse(check_line_circle_intersection(line, circle))

        line = Line(Point(1, 1), Point(3, 4))
        circle = Circle(Point(2, 2), 5)
        self.assertTrue(check_line_circle_intersection(line, circle))

        line = Line(Point(-3, 2), Point(3, 2))
        circle = Circle(Point(0, 0), 2)
        self.assertTrue(check_line_circle_intersection(line, circle))

    def test_calculate_line_circle_intersection_points(self):
        """Test the calculate_line_circle_intersection_points function."""
        # Case 1: The line segment intersects the circle in two points
        line = Line(Point(-8, 3), Point(8, 3))
        circle = Circle(Point(0, 0), 5)
        points = calculate_line_circle_intersection_points(line, circle)
        self.assertEqual(len(points), 2)
        self.assertIn(Point(-4, 3), points)
        self.assertIn(Point(4, 3), points)

        line = Line(Point(-8, -6), Point(8, 6))
        circle = Circle(Point(0, 0), 5)
        points = calculate_line_circle_intersection_points(line, circle)
        self.assertEqual(len(points), 2)
        self.assertIn(Point(-4, -3), points)
        self.assertIn(Point(4, 3), points)

        # Case 2: The line segment intersects the circle in one point
        line = Line(Point(0, 0), Point(8, 6))
        circle = Circle(Point(0, 0), 5)
        points = calculate_line_circle_intersection_points(line, circle)
        self.assertEqual(len(points), 1)
        self.assertIn(Point(4, 3), points)

        line = Line(Point(0, 0), Point(8, 6))
        circle = Circle(Point(8, 6), 5)
        points = calculate_line_circle_intersection_points(line, circle)
        self.assertEqual(len(points), 1)
        self.assertIn(Point(4, 3), points)

        # Case 3: The line segment is tangent to the circle
        line = Line(Point(-8, 3), Point(8, 3))
        circle = Circle(Point(0, 0), 3)
        points = calculate_line_circle_intersection_points(line, circle)
        self.assertEqual(len(points), 1)
        self.assertIn(Point(0, 3), points)

        line = Line(Point(-2, -6), Point(-2, 3))
        circle = Circle(Point(0, 0), 2)
        points = calculate_line_circle_intersection_points(line, circle)
        self.assertEqual(len(points), 1)
        self.assertIn(Point(-2, 0), points)

        # Case 4: The line segment does not intersect the circle
        line = Line(Point(-8, 3), Point(-4, 3))
        circle = Circle(Point(0, 0), 1)
        points = calculate_line_circle_intersection_points(line, circle)
        self.assertEqual(len(points), 0)

        line = Line(Point(8, 3), Point(8, -6))
        circle = Circle(Point(0, 0), 0.5)
        points = calculate_line_circle_intersection_points(line, circle)
        self.assertEqual(len(points), 0)


if __name__ == "__main__":
    unittest.main()
