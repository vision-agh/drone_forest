#include <drone_forest/geometric/line.h>

namespace evs
{
namespace geometric
{

Line::Line() : start_{0, 0}, end_{0, 0} {}

Line::Line(Point start, Point end) : start_{start}, end_{end} {}

Point Line::Start() const
{
  return start_;
}

Point Line::End() const
{
  return end_;
}

std::vector<Point> Line::CalculateCircleIntersection(const Circle& circ) const
{
  // If the line is a point
  if (start_ == end_)
  {
    if (start_.Distance(circ.Center()) == circ.Radius())
    {
      return {start_};
    }

    return {};
  }

  Point line_closest_point_ = CalculateClosestPoint(circ.Center());
  double distance_to_line_ = line_closest_point_.Distance(circ.Center());

  // The line ist tangent to the circle
  if (distance_to_line_ == circ.Radius())
  {
    return {line_closest_point_};
  }

  // The line does not intersect the circle
  if (distance_to_line_ > circ.Radius())
  {
    return {};
  }

  // The line intersects the circle
  double dx = end_.x() - start_.x();
  double dy = end_.y() - start_.y();
  double fx = start_.x() - circ.Center().x();
  double fy = start_.y() - circ.Center().y();

  double a = dx * dx + dy * dy;
  double b = 2 * (fx * dx + fy * dy);
  double c = fx * fx + fy * fy - circ.Radius() * circ.Radius();

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

void Line::Draw(cv::Mat& image, const cv::Scalar& color, Point t_vec,
                double m2px) const
{
  int img_height = image.rows;
  cv::line(image, start_.ToCvPoint(t_vec, m2px, img_height),
           end_.ToCvPoint(t_vec, m2px, img_height), color);
}

}  // namespace geometric
}  // namespace evs