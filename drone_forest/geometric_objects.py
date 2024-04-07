"""Basic geometric objects for the drone forest simulation."""

import dataclasses
from typing import List


@dataclasses.dataclass
class Point:
    """A point in 2D space."""

    x: float
    y: float

    def __add__(self, other: "Point") -> List[float]:
        """Add two points together."""
        return [self.x + other.x, self.y + other.y]

    def __sub__(self, other: "Point") -> List[float]:
        """Subtract one point from another."""
        return [self.x - other.x, self.y - other.y]

    def __truediv__(self, coeff: float) -> "Point":
        """Divide point by coefficient."""
        return Point(self.x / coeff, self.y / coeff)

    def __mul__(self, coeff: float) -> "Point":
        """Multiply point by coefficient."""
        return Point(self.x * coeff, self.y * coeff)


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
