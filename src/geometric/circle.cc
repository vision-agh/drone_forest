#include <drone_forest/geometric/circle.h>

namespace evs
{
namespace geometric
{

Circle::Circle() : center_{0, 0}, radius_{0} {}

Circle::Circle(Point center, double radius) : center_{center}, radius_{radius}
{
}

Point Circle::Center() const
{
  return center_;
}

double Circle::Radius() const
{
  return radius_;
}

void Circle::Draw(cv::Mat& image, const cv::Scalar& color, Point t_vec,
                  double m2px) const
{
  int img_height = image.rows;
  cv::circle(image, center_.ToCvPoint(t_vec, m2px, img_height), radius_ * m2px,
             color, cv::FILLED);
}

void Circle::UpdatePosition(Point new_center)
{
  center_ = new_center;
}

}  // namespace geometric
}  // namespace evs