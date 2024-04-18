#ifndef _DRONE_FOREST_GEOMETRIC_H_
#define _DRONE_FOREST_GEOMETRIC_H_

#include <cmath>
#include <nlohmann/json.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <vector>

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
void to_json(json& j, const Point& p)
{
  j = {{"x", p.x()}, {"y", p.y()}};
}

/**
 * @brief Helper function to convert a JSON input to a Point object
 *
 * @param j JSON object to read from
 * @param p Point object to fill
 */
void from_json(const json& j, Point& p)
{
  p = Point(j["x"], j["y"]);
}

/**
 * @brief A simple 2D circle class
 *
 */
class Circle
{
 public:
  /**
   * @brief Construct a default Circle object
   *
   */
  Circle() : center_(0, 0), radius_(0) {}

  /**
   * @brief Construct a new Circle object
   *
   * @param center Center of the circle
   * @param radius Radius of the circle
   */
  Circle(Point center, double radius) : center_(center), radius_(radius) {}

  /**
   * @brief Get the center of the circle
   *
   * @return Point Center of the circle
   */
  Point center() const
  {
    return center_;
  }

  /**
   * @brief Get the radius of the circle
   *
   * @return double Radius of the circle
   */
  double radius() const
  {
    return radius_;
  }

  /**
   * @brief Draw the circle on an image
   *
   * @param image Image to draw the circle on
   * @param color Color of the circle
   * @param t_vec Translation vector
   * @param m2px Meters to pixels conversion factor
   */
  void Draw(cv::Mat& image, cv::Scalar color, Point t_vec, double m2px) const;

 private:
  Point center_;
  double radius_;
};

/**
 * @brief A simple 2D line class
 *
 * The line is defined by two points, a start and an end point.
 */
class Line
{
 public:
  /**
   * @brief Construct a default Line object
   *
   */
  Line() : start_(0, 0), end_(0, 0) {}

  /**
   * @brief Construct a new Line object
   *
   * @param start Start point of the line
   * @param end End point of the line
   */
  Line(Point start, Point end) : start_(start), end_(end) {}

  /**
   * @brief Get the start point of the line
   *
   * @return Point Start point of the line
   */
  Point start() const
  {
    return start_;
  }

  /**
   * @brief Get the end point of the line
   *
   * @return Point End point of the line
   */
  Point end() const
  {
    return end_;
  }

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
  void Draw(cv::Mat& image, cv::Scalar color, Point t_vec, double m2px) const;

 private:
  Point start_;
  Point end_;
};

}  // namespace geometric
}  // namespace evs

#endif  // _DRONE_FOREST_GEOMETRIC_H_