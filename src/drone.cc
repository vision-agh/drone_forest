#include <drone_forest/drone.h>

namespace evs
{
namespace drone
{

void Drone::Draw(cv::Mat& image, geometric::Point t_vec, double m2px) const
{
  // Draw the LiDAR sensor
  lidar_.Draw(image, t_vec, m2px);
}

void Drone::Move(double dt, const geometric::Point& velocity)
{
  // Calculate the acceleration
  geometric::Point acceleration = (velocity - velocity_) / dt;
  if (acceleration.x() > max_acceleration_)
  {
    acceleration = geometric::Point(max_acceleration_, acceleration.y());
  }
  if (acceleration.y() > max_acceleration_)
  {
    acceleration = geometric::Point(acceleration.x(), max_acceleration_);
  }
  if (acceleration.x() < -max_acceleration_)
  {
    acceleration = geometric::Point(-max_acceleration_, acceleration.y());
  }
  if (acceleration.y() < -max_acceleration_)
  {
    acceleration = geometric::Point(acceleration.x(), -max_acceleration_);
  }

  // Calculate the new velocity
  velocity_ = velocity_ + acceleration * dt;
  if (velocity.x() > max_speed_)
  {
    velocity_ = geometric::Point(max_speed_, velocity_.y());
  }
  if (velocity.y() > max_speed_)
  {
    velocity_ = geometric::Point(velocity_.x(), max_speed_);
  }
  if (velocity.x() < -max_speed_)
  {
    velocity_ = geometric::Point(-max_speed_, velocity_.y());
  }
  if (velocity.y() < -max_speed_)
  {
    velocity_ = geometric::Point(velocity_.x(), -max_speed_);
  }

  // Calculate the new position
  geometric::Point new_position = position_ + velocity * dt;

  // Update the LiDAR sensor position
  lidar_.UpdatePosition(new_position);

  // Update the drone position
  position_ = new_position;
}

}  // namespace drone
}  // namespace evs