"""Script for the lidar class."""

import cv2
import math
import numpy as np
from typing import List

from drone_forest.geometric_objects import Circle, Line, Point
from drone_forest.geometric_utils import (
    calculate_line_circle_intersection_points,
    distance,
)
from drone_forest.render_utils import transform_coords_to_render, IMAGE_HEIGHT


class Lidar:
    """A lidar object in the drone forest.

    Attributes:
        position (Point): The position of the lidar.
        max_range (float): The maximum range of the lidar.
        beam_angles (List[float]): The angles of the beams emitted by the lidar.
        beam_lines (List[Line]): The lines of the beams emitted by the lidar.
    """

    def __init__(self, position: Point, max_range: float, n_beams: int):
        """Initialize the lidar object.

        Args:
            position (Point): The position of the lidar.
            max_range (float): The maximum range of the lidar.
            n_beams (int): The number of beams emitted by the lidar.
        """
        assert max_range >= 0, "The maximum range of the lidar must be non-negative."
        assert n_beams > 0, "The number of beams must be positive."

        self.position = position
        self.max_range = max_range
        self.beam_angles = [i * 2 * math.pi / n_beams for i in range(n_beams)]
        self.beam_lines: List[Line] = []
        for angle in self.beam_angles:
            end = Point(
                self.position.x + self.max_range * math.cos(angle),
                self.position.y + self.max_range * math.sin(angle),
            )
            self.beam_lines.append(Line(self.position, end))

    def draw(self, img: np.ndarray, t_vec: Point, m2px: float):
        """Draw the lidar on the given axis.

        Args:
            ax (matplotlib.axes.Axes): The axis to draw the lidar on.
        """
        assert img is not None, "The axis must be provided."

        # Draw the beams emitted by the lidar
        for line in self.beam_lines:
            coord_start = transform_coords_to_render(line.start, t_vec)
            coord_end = transform_coords_to_render(line.end, t_vec)
            cv2.line(
                img,
                (
                    int(coord_start.x * m2px),
                    int(coord_start.y * m2px) + IMAGE_HEIGHT,
                ),
                (
                    int(coord_end.x * m2px),
                    int(coord_end.y * m2px) + IMAGE_HEIGHT,
                ),
                (255, 0, 0),
                1,
            )

    def scan(self, obstacles: List[Circle]) -> List[float]:
        """Scan the environment for obstacles.

        Args:
            obstacles (List[Circle]): The list of obstacles in the environment.

        Returns:
            List[float]: The list of distances to the obstacles.
        """
        distances = []
        for idx, angle in enumerate(self.beam_angles):
            end = Point(
                self.position.x + self.max_range * math.cos(angle),
                self.position.y + self.max_range * math.sin(angle),
            )
            line = Line(self.position, end)
            dist = self.max_range
            for obstacle in obstacles:
                intersection_points = calculate_line_circle_intersection_points(
                    line, obstacle
                )
                if intersection_points:
                    # Find the closest intersection point
                    intersection_point = intersection_points[0]
                    for point in intersection_points:
                        if distance(self.position, point) < distance(
                            self.position, intersection_point
                        ):
                            intersection_point = point

                    # Check if the intersection point is not hide by other obstacles
                    if dist == self.max_range:
                        dist = distance(self.position, intersection_point)
                    else:
                        dist = min(dist, distance(self.position, intersection_point))
                    end_point = Point(
                        self.position.x + dist * math.cos(angle),
                        self.position.y + dist * math.sin(angle),
                    )
                    line = Line(self.position, end_point)
            self.beam_lines[idx] = line
            distances.append(dist)
        return distances
