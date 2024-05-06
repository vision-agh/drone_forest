#include <drone_forest/geometric.h>

namespace evs
{
namespace geometric
{

cv::Point Point::ToCvPoint(Point t_vec, double m2pix, int img_height) const
{
  // Translate the point to the Image origin
  Point t_coords{x_ + t_vec.x(), y_ + t_vec.y()};

  // Scale the point to the Image coordinate system
  Point s_coords = t_coords * m2pix;

  // Flip the point to the Image coordinate system
  Point f_coords{s_coords.x(), img_height - 1 - s_coords.y()};

  return cv::Point(std::round(f_coords.x()), std::round(f_coords.y()));
}

void Circle::Draw(cv::Mat& image, cv::Scalar color, Point t_vec,
                  double m2px) const
{
  int img_height = image.rows;
  cv::circle(image, center_.ToCvPoint(t_vec, m2px, img_height), radius_ * m2px,
             color, cv::FILLED);
}

std::vector<Point> Line::CalculateCircleIntersection(const Circle& circ) const
{
  // If the line is a point
  if (start_ == end_)
  {
    if (start_.Distance(circ.center()) == circ.radius())
    {
      return {start_};
    }

    return {};
  }

  Point line_closest_point_ = CalculateClosestPoint(circ.center());
  double distance_to_line_ = line_closest_point_.Distance(circ.center());

  // The line ist tangent to the circle
  if (distance_to_line_ == circ.radius())
  {
    return {line_closest_point_};
  }

  // The line does not intersect the circle
  if (distance_to_line_ > circ.radius())
  {
    return {};
  }

  // The line intersects the circle
  double dx = end_.x() - start_.x();
  double dy = end_.y() - start_.y();
  double fx = start_.x() - circ.center().x();
  double fy = start_.y() - circ.center().y();

  double a = dx * dx + dy * dy;
  double b = 2 * (fx * dx + fy * dy);
  double c = fx * fx + fy * fy - circ.radius() * circ.radius();

  double discriminant = b * b - 4 * a * c;
  if (discriminant < 0)
  {
    return {};
  }

  double discriminant_sqrt = sqrt(discriminant);
  double t1 = (-b + discriminant_sqrt) / (2 * a);
  double t2 = (-b - discriminant_sqrt) / (2 * a);

  if (0 <= t1 && t1 <= 1 && 0 <= t2 && t2 <= 1)
  {
    return {Point(start_.x() + t1 * dx, start_.y() + t1 * dy),
            Point(start_.x() + t2 * dx, start_.y() + t2 * dy)};
  }
  else if (0 <= t1 && t1 <= 1)
  {
    return {Point(start_.x() + t1 * dx, start_.y() + t1 * dy)};
  }
  else if (0 <= t2 && t2 <= 1)
  {
    return {Point(start_.x() + t2 * dx, start_.y() + t2 * dy)};
  }
  else
  {
    return {};
  }
}

Point Line::CalculateClosestPoint(const Point& pt) const
{
  double dx = end_.x() - start_.x();
  double dy = end_.y() - start_.y();
  double t = ((pt.x() - start_.x()) * dx + (pt.y() - start_.y()) * dy)
             / (dx * dx + dy * dy);

  if (t < 0)
  {
    return start_;
  }
  else if (t > 1)
  {
    return end_;
  }
  else
  {
    return Point(start_.x() + t * dx, start_.y() + t * dy);
  }
}

void Line::Draw(cv::Mat& image, cv::Scalar color, Point t_vec,
                double m2px) const
{
  int img_height = image.rows;
  cv::line(image, start_.ToCvPoint(t_vec, m2px, img_height),
           end_.ToCvPoint(t_vec, m2px, img_height), color);
}

bool Rectangle::CheckCircleIntersection(const Circle& circ) const
{
  // Check if the circle is inside the rectangle
  if (top_left_.x() <= circ.center().x()
      && circ.center().x() <= bottom_right_.x()
      && bottom_right_.y() <= circ.center().y()
      && circ.center().y() <= top_left_.y())
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

void Rectangle::Draw(cv::Mat& image, cv::Scalar color, Point t_vec,
                     double m2px) const
{
  int img_height = image.rows;
  cv::Point top_left = top_left_.ToCvPoint(t_vec, m2px, img_height);
  cv::Point bottom_right = bottom_right_.ToCvPoint(t_vec, m2px, img_height);
  cv::rectangle(image, top_left, bottom_right, color, cv::FILLED);
}

void Rectangle::UpdatePosition(const Point& new_center)
{
  Point half_diagonal = (bottom_right_ - top_left_) / 2;
  top_left_ = new_center + half_diagonal;
  bottom_right_ = new_center - half_diagonal;
}

}  // namespace geometric
}  // namespace evs