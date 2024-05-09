#ifndef _DRONE_FOREST_GEOMETRIC_LINE_H_
#define _DRONE_FOREST_GEOMETRIC_LINE_H_

#include <vector>

#include "circle.h"
#include "shape.h"

namespace evs
{
namespace geometric
{

/**
 * @brief A simple 2D line class
 *
 * The line is defined by two points, a start and an end point.
 */
class Line : public Shape
{
 public:
  /**
   * @brief Construct a default Line object
   *
   */
  Line();

  /**
   * @brief Construct a new Line object
   *
   * @param start Start point of the line
   * @param end End point of the line
   */
  Line(Point start, Point end);

  /**
   * @brief Get the start point of the line
   *
   * @return Point Start point of the line
   */
  Point Start() const;

  /**
   * @brief Get the end point of the line
   *
   * @return Point End point of the line
   */
  Point End() const;

  /**
   * @brief Calculate the intersection points of a line and a given circle
   *
   * Algorithm from https://stackoverflow.com/a/1084899
   *
   * @param circ Circle to intersect with
   * @return std::vector<Point> Intersection points
   */
  std::vector<Point> CalculateCircleIntersection(const Circle& circ) const;

  /**
   * @brief Calculate the closest point on the line to a given point
   *
   * Algorithm from
   * https://web.archive.org/web/20210507021429/https://geomalgorithms.com/a02-_lines.html
   *
   * @param pt Point to calculate the closest point to
   * @return Point Closest point on the line
   */
  Point CalculateClosestPoint(const Point& pt) const;

  /**
   * @brief Draw the line on an image
   *
   * @param image Image to draw the line on
   * @param color Color of the line
   * @param t_vec Translation vector
   * @param m2px Meters to pixels conversion factor
   */
  void Draw(cv::Mat& image, const cv::Scalar& color, Point t_vec,
            double m2px) const override;

 private:
  Point start_;
  Point end_;
};

}  // namespace geometric
}  // namespace evs

#endif  // _DRONE_FOREST_GEOMETRIC_LINE_H_