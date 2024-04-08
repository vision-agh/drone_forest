#include <drone_forest/geometric.h>

namespace evs
{
namespace geometric
{

// void Circle::Draw(cv::Mat& img, cv::Scalar color) const
// {
//   cv::circle(img, cv::Point(center_.x(), center_.y()), radius_, color,
//              cv::FILLED);
// }

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

}  // namespace geometric
}  // namespace evs