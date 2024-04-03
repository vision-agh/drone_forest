"""Basic geometric objects for the drone forest simulation."""

import dataclasses


@dataclasses.dataclass
class Point:
    """A point in 2D space."""

    x: float
    y: float


@dataclasses.dataclass
class Line:
    """A line segment in 2D space."""

    start: Point
    end: Point


@dataclasses.dataclass
class Circle:
    """A circle in 2D space."""

    center: Point
    radius: float
