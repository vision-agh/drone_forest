#include <drone_forest/geometric/point.h>

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

void to_json(json& j, const Point& p)
{
  j = {{"x", p.x()}, {"y", p.y()}};
}

void from_json(const json& j, Point& p)
{
  p = Point(j.at("x").get<double>(), j.at("y").get<double>());
}

}  // namespace geometric
}  // namespace evs