#ifndef _DRONE_FOREST_DRONE_H_
#define _DRONE_FOREST_DRONE_H_

#include <drone_forest/geometric.h>
#include <drone_forest/lidar.h>

namespace evs
{
namespace drone
{

class Drone
{
 public:
  Drone(const geometric::Point& position, double lidar_range, int lidar_n_beams,
        double max_speed = 1.0, double max_acceleration = 0.6)
      : position_(position),
        velocity_(0, 0),
        max_speed_(max_speed),
        max_acceleration_(max_acceleration),
        lidar_(position, lidar_range, lidar_n_beams)
  {
  }

  geometric::Point position() const
  {
    return position_;
  }

  lidar::Lidar lidar() const
  {
    return lidar_;
  }

  void Draw(cv::Mat& image, geometric::Point t_vec, double m2px) const;

  void Move(double dt, const geometric::Point& velocity);

 private:
  geometric::Point position_;
  geometric::Point velocity_;
  double max_speed_;
  double max_acceleration_;
  lidar::Lidar lidar_;
};

}  // namespace drone
}  // namespace evs

#endif  // _DRONE_FOREST_DRONE_H_