#include <drone_forest/geometric/rectangle.h>

namespace evs
{
namespace geometric
{

Rectangle::Rectangle() : top_left_(0, 0), bottom_right_(0, 0) {}

Rectangle::Rectangle(Point top_left, Point bottom_right)
    : top_left_(top_left),
      bottom_right_(bottom_right)
{
}

Point Rectangle::BottomRight() const
{
  return bottom_right_;
}

bool Rectangle::CheckCircleIntersection(const Circle& circ) const
{
  // Check if the circle is inside the rectangle
  if (top_left_.x() <= circ.Center().x()
      && circ.Center().x() <= bottom_right_.x()
      && bottom_right_.y() <= circ.Center().y()
      && circ.Center().y() <= top_left_.y())
  {
    return true;
  }

  // Check if the circle intersects the rectangle
  Line top_line{top_left_, Point(bottom_right_.x(), top_left_.y())};
  std::vector<Point> top_line_intersections =
      top_line.CalculateCircleIntersection(circ);
  if (!top_line_intersections.empty())
  {
    return true;
  }

  Line bottom_line{Point(top_left_.x(), bottom_right_.y()), bottom_right_};
  std::vector<Point> bottom_line_intersections =
      bottom_line.CalculateCircleIntersection(circ);
  if (!bottom_line_intersections.empty())
  {
    return true;
  }

  Line left_line{top_left_, Point(top_left_.x(), bottom_right_.y())};
  std::vector<Point> left_line_intersections =
      left_line.CalculateCircleIntersection(circ);
  if (!left_line_intersections.empty())
  {
    return true;
  }

  Line right_line{Point(bottom_right_.x(), top_left_.y()), bottom_right_};
  std::vector<Point> right_line_intersections =
      right_line.CalculateCircleIntersection(circ);
  if (!right_line_intersections.empty())
  {
    return true;
  }

  return false;
}

void Rectangle::Draw(cv::Mat& image, const cv::Scalar& color, Point t_vec,
                     double m2px) const
{
  int img_height = image.rows;
  cv::Point top_left = top_left_.ToCvPoint(t_vec, m2px, img_height);
  cv::Point bottom_right = bottom_right_.ToCvPoint(t_vec, m2px, img_height);
  cv::rectangle(image, top_left, bottom_right, color, cv::FILLED);
}

Point Rectangle::TopLeft() const
{
  return top_left_;
}

void Rectangle::UpdatePosition(Point new_center)
{
  Point half_diagonal = (bottom_right_ - top_left_) / 2;
  top_left_ = new_center + half_diagonal;
  bottom_right_ = new_center - half_diagonal;
}

}  // namespace geometric
}  // namespace evs