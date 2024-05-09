#ifndef _DRONE_FOREST_GEOMETRIC_CIRCLE_H_
#define _DRONE_FOREST_GEOMETRIC_CIRCLE_H_

#include "shape.h"

namespace evs
{
namespace geometric
{

/**
 * @brief A simple 2D circle class
 *
 */
class Circle : public Shape
{
 public:
  /**
   * @brief Construct a default Circle object
   *
   */
  Circle();

  /**
   * @brief Construct a new Circle object
   *
   * @param center Center of the circle
   * @param radius Radius of the circle
   */
  Circle(Point center, double radius);

  /**
   * @brief Get the center of the circle
   *
   * @return Point Center of the circle
   */
  Point Center() const;

  /**
   * @brief Get the radius of the circle
   *
   * @return double Radius of the circle
   */
  double Radius() const;

  /**
   * @brief Draw the circle on an image
   *
   * @param image Image to draw the circle on
   * @param color Color of the circle
   * @param t_vec Translation vector
   * @param m2px Meters to pixels conversion factor
   */
  void Draw(cv::Mat& image, const cv::Scalar& color, Point t_vec,
            double m2px) const override;

 private:
  Point center_;
  double radius_;
};

}  // namespace geometric
}  // namespace evs

#endif  // _DRONE_FOREST_GEOMETRIC_CIRCLE_H_