"""Some geometric utilities for the drone forest project."""

from .geometric_objects import Circle, Line, Point

from typing import List


def calculate_closest_point_on_line_segment(line: Line, point: Point) -> Point:
    """Calculate the closest point on a line segment to a given point.

    This function uses the algorithm from
    https://web.archive.org/web/20210507021429/https://geomalgorithms.com/a02-_lines.html

    Args:
        line (Line): The line segment.
        point (Point): The point.

    Returns:
        Point: The closest point on the line segment to the point.
    """
    x1, y1 = line.start.x, line.start.y
    x2, y2 = line.end.x, line.end.y
    x3, y3 = point.x, point.y

    # Calculate the closest point on the line segment to the point
    px = x2 - x1
    py = y2 - y1
    p_dot_product = px * px + py * py
    u = ((x3 - x1) * px + (y3 - y1) * py) / p_dot_product

    # The closest point is on the line segment
    if u > 1:
        u = 1
    elif u < 0:
        u = 0

    return Point(x1 + u * px, y1 + u * py)


def calculate_line_circle_intersection_points(
    line: Line, circle: Circle
) -> List[Point]:
    """Calculate the intersection points between a line segment and a circle.

    Solution from https://stackoverflow.com/a/1084899

    Args:
        line (Line): The line segment.
        circle (Circle): The circle.

    Returns:
        List[Point]: The intersection points between the line segment and the circle.
    """
    # The line segment is a point if the start and end points are the same
    if line.start == line.end:
        if distance(line.start, circle.center) == circle.radius:
            return [line.start]
        return []

    # The line segment is a line
    line_closest_point = calculate_closest_point_on_line_segment(line, circle.center)
    d = distance(line_closest_point, circle.center)

    # The line segment is tangent to the circle
    if d == circle.radius:
        return [line_closest_point]

    # The line segment does not intersect the circle
    if d > circle.radius:
        return []

    # The line segment can intersect the circle at two points or one point
    # if the line segment is inside the circle
    x1, y1 = line.start.x, line.start.y
    x2, y2 = line.end.x, line.end.y
    x3, y3 = circle.center.x, circle.center.y

    dx = x2 - x1
    dy = y2 - y1
    fx = x1 - x3
    fy = y1 - y3

    a = dx * dx + dy * dy
    b = 2 * (dx * fx + dy * fy)
    c = fx * fx + fy * fy - circle.radius**2

    discriminant = b * b - 4 * a * c
    if discriminant < 0:
        return []

    t1 = (-b + discriminant**0.5) / (2 * a)
    t2 = (-b - discriminant**0.5) / (2 * a)

    if 0 <= t1 <= 1 and 0 <= t2 <= 1:
        return [
            Point(x1 + t1 * dx, y1 + t1 * dy),
            Point(x1 + t2 * dx, y1 + t2 * dy),
        ]
    elif 0 <= t1 <= 1:
        return [Point(x1 + t1 * dx, y1 + t1 * dy)]
    elif 0 <= t2 <= 1:
        return [Point(x1 + t2 * dx, y1 + t2 * dy)]
    else:
        return []


def check_line_circle_intersection(line: Line, circle: Circle) -> bool:
    """Check if a line segment intersects a circle.

    Args:
        line (Line): The line segment.
        circle (Circle): The circle.

    Returns:
        bool: True if the line segment intersects the circle.
    """
    # The line segment is a point if the start and end points are the same
    if line.start == line.end:
        return distance(line.start, circle.center) <= circle.radius

    # The line segment is a line
    line_closest_point = calculate_closest_point_on_line_segment(line, circle.center)
    d = distance(line_closest_point, circle.center)

    return d <= circle.radius


def distance(p1: Point, p2: Point) -> float:
    """Compute the Euclidean distance between two points.

    Args:
        p1 (Point): First point.
        p2 (Point): Second point.

    Returns:
        float: The Euclidean distance between the two points.
    """
    return ((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2) ** 0.5
