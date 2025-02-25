#ifndef _DRONE_FOREST_GEOMETRIC_POINT_H_
#define _DRONE_FOREST_GEOMETRIC_POINT_H_

#include <cmath>
#include <nlohmann/json.hpp>
#include <opencv4/opencv2/opencv.hpp>

using json = nlohmann::json;

namespace evs
{
namespace geometric
{

/**
 * @brief A simple 2D point class
 *
 */
class Point
{
 public:
  /**
   * @brief Construct a default Point object at the origin
   *
   */
  Point() : x_(0), y_(0) {}

  /**
   * @brief Construct a new Point object
   *
   * @param x x-coordinate of the point
   * @param y y-coordinate of the point
   */
  Point(double x, double y) : x_(x), y_(y) {}

  /**
   * @brief Get the x-coordinate of the point
   *
   * @return double x-coordinate
   */
  double x() const
  {
    return x_;
  }

  /**
   * @brief Get the y-coordinate of the point
   *
   * @return double y-coordinate
   */
  double y() const
  {
    return y_;
  }

  /**
   * @brief Overload the addition operator to add two points
   *
   * Calculate the simple sum over the x and y coordinates.
   *
   * @param p Point to add
   * @return Point Sum of the two points
   */
  Point operator+(const Point& p) const
  {
    return Point(x_ + p.x_, y_ + p.y_);
  }

  /**
   * @brief Overload the subtraction operator to subtract two points
   *
   * Calculate the simple difference over the x and y coordinates.
   *
   * @param p Point to subtract
   * @return Point Difference of the two points
   */
  Point operator-(const Point& p) const
  {
    return Point(x_ - p.x_, y_ - p.y_);
  }

  /**
   * @brief Overload the multiplication operator to multiply a point by a scalar
   *
   * Multiply the x and y coordinates by the scalar.
   *
   * @param s Scalar to multiply by
   * @return Point Product of the point and the scalar
   */
  Point operator*(double s) const
  {
    return Point(x_ * s, y_ * s);
  }

  /**
   * @brief Overload the division operator to divide a point by a scalar
   *
   * Divide the x and y coordinates by the scalar.
   *
   * @param s Scalar to divide by
   * @return Point Quotient of the point and the scalar
   */
  Point operator/(double s) const
  {
    return Point(x_ / s, y_ / s);
  }

  /**
   * @brief Overload the equality operator to compare two points
   *
   * @param p Point to compare to
   * @return true Points are equal
   * @return false Points are not equal
   */
  bool operator==(const Point& p) const
  {
    return x_ == p.x_ && y_ == p.y_;
  }

  /**
   * @brief Overload the inequality operator to compare two points
   *
   * @param p Point to compare to
   * @return true Points are not equal
   * @return false Points are equal
   */
  bool operator!=(const Point& p) const
  {
    return !(*this == p);
  }

  /**
   * @brief Calculate the distance between two points
   *
   * @param p Point to calculate the distance to
   *
   * @return double Distance between the two points
   */
  double Distance(const Point& p) const
  {
    return sqrt(pow(x_ - p.x_, 2) + pow(y_ - p.y_, 2));
  }

  /**
   * @brief Convert the point to a cv::Point
   *
   * This method is useful to convert the point to a cv::Point for drawing
   * operations.
   *
   * @param t_vec Translation vector
   * @param m2pix Meters to pixels conversion factor
   * @param img_height Height of the image
   * @return cv::Point Converted point
   */
  cv::Point ToCvPoint(Point t_vec, double m2pix, int img_height) const;

 private:
  double x_;
  double y_;
};

/**
 * @brief Helper function to convert a Point object to a JSON input
 *
 * @param j JSON object to fill
 * @param p Point object to convert
 */
void to_json(json& j, const Point& p);

/**
 * @brief Helper function to convert a JSON input to a Point object
 *
 * @param j JSON object to read from
 * @param p Point object to fill
 */
void from_json(const json& j, Point& p);

}  // namespace geometric
}  // namespace evs

#endif  // _DRONE_FOREST_GEOMETRIC_POINT_H_