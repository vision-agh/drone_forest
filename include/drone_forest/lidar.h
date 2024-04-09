#ifndef _DRONE_FOREST_LIDAR_H_
#define _DRONE_FOREST_LIDAR_H_

#include <drone_forest/geometric.h>

#include <cmath>
#include <vector>

namespace evs
{
namespace lidar
{

class Lidar
{
 public:
  Lidar(const geometric::Point& position, double range, int n_beams);

  geometric::Point position() const
  {
    return position_;
  }

  double range() const
  {
    return range_;
  }

  std::vector<double> angles() const
  {
    return beam_angles_;
  }

  std::vector<geometric::Line> beams() const
  {
    return beams_;
  }

  void Draw(cv::Mat& image, geometric::Point t_vec, double m2px) const;

  std::vector<double> Scan(const std::vector<geometric::Circle>& obstacles);

  geometric::Point UpdatePosition(const geometric::Point& new_position)
  {
    geometric::Point old_position = position_;
    position_ = new_position;
    return old_position;
  }

 private:
  geometric::Point position_;
  double range_;
  std::vector<double> beam_angles_;
  std::vector<geometric::Line> beams_;
};

}  // namespace lidar
}  // namespace evs

#endif  // _DRONE_FOREST_LIDAR_H_