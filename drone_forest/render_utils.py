"""Script containing utility functions for rendering the drone forest environment."""

from drone_forest.geometric_objects import Point

IMAGE_HEIGHT = 800


def transform_coords_to_render(world_coords: Point, t_vec: Point) -> Point:
    """Transform world coordinates to render coordinates.

    Args:
        world_coords (Point): The world coordinates to transform.
        m2px (float): The meters to pixels conversion factor.

    Returns:
        Point: The transformed render coordinates.
    """
    t_coords = world_coords + t_vec
    return Point(t_coords[0], -t_coords[1])
