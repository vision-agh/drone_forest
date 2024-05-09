#ifndef _DRONE_FOREST_GEOMETRIC_SHAPE_H_
#define _DRONE_FOREST_GEOMETRIC_SHAPE_H_

#include <opencv4/opencv2/opencv.hpp>

#include "point.h"

namespace evs
{
namespace geometric
{

/**
 * @brief A simple 2D shape class
 *
 * This class is the base class for all geometric shapes.
 */
class Shape
{
 public:
  /**
   * @brief Construct a default Shape object
   *
   */
  Shape() = default;

  /**
   * @brief Destroy the Shape object
   *
   */
  virtual ~Shape() = default;

  /**
   * @brief Draw the shape on an image
   *
   * @param image Image to draw the shape on
   * @param color Color of the shape
   * @param t_vec Translation vector
   * @param m2px Meters to pixels conversion factor
   */
  virtual void Draw(cv::Mat& image, const cv::Scalar& color, Point t_vec,
                    double m2px) const = 0;
};

}  // namespace geometric
}  // namespace evs

#endif  // _DRONE_FOREST_GEOMETRIC_SHAPE_H_