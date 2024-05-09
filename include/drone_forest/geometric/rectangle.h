#ifndef _DRONE_FOREST_GEOMETRIC_RECTANGLE_H_
#define _DRONE_FOREST_GEOMETRIC_RECTANGLE_H_

#include <vector>

#include "circle.h"
#include "line.h"
#include "point.h"
#include "shape.h"

namespace evs
{
namespace geometric
{

/**
 * @brief Rectangle shape
 *
 * A rectangle is defined by two points: the top left corner and the bottom
 * right corner.
 *
 */
class Rectangle : public Shape
{
 public:
  /**
   * @brief Construct a default Rectangle object
   *
   */
  Rectangle();

  /**
   * @brief Construct a new Rectangle object
   *
   * @param top_left Top left corner of the rectangle
   * @param bottom_right Bottom right corner of the rectangle
   */
  Rectangle(Point top_left, Point bottom_right);

  /**
   * @brief Get the bottom right corner of the rectangle
   *
   * @return Point Bottom right corner of the rectangle
   */
  Point BottomRight() const;

  /**
   * @brief Determine if any part of the rectangle intersects with a given
   * circle
   *
   * @param circ Circle to check for intersection
   * @return true If the rectangle intersects with the circle
   * @return false If the rectangle does not intersect with the circle
   */
  bool CheckCircleIntersection(const Circle& circ) const;

  /**
   * @brief Draw the rectangle on an image
   *
   * @param image Image to draw the rectangle on
   * @param color Color of the rectangle
   * @param t_vec Translation vector to apply to the rectangle
   * @param m2px Meters to pixels conversion factor
   */
  void Draw(cv::Mat& image, const cv::Scalar& color, Point t_vec,
            double m2px) const override;

  /**
   * @brief Get the top left corner of the rectangle
   *
   * @return Point Top left corner of the rectangle
   */
  Point TopLeft() const;

  /**
   * @brief Update the position of the rectangle
   *
   * @param new_center New center of the rectangle
   */
  void UpdatePosition(const Point& new_center);

 private:
  Point top_left_;
  Point bottom_right_;
};

}  // namespace geometric
}  // namespace evs

#endif  // _DRONE_FOREST_GEOMETRIC_RECTANGLE_H_