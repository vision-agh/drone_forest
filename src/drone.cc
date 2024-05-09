#include <drone_forest/drone.h>

namespace evs
{
namespace drone
{

Drone::Drone(const geometric::Point& position, double lidar_range,
             int lidar_n_beams, double max_speed, double max_acceleration,
             double drone_width_m, double drone_height_m)
    : position_(position),
      velocity_(0.0, 0.0),
      max_speed_(max_speed),
      max_acceleration_(max_acceleration),
      lidar_(position, lidar_range, lidar_n_beams),
      body_(position + geometric::Point(drone_width_m / 2, drone_height_m / 2),
            position - geometric::Point(drone_width_m / 2, drone_height_m / 2))
{
}

geometric::Rectangle Drone::Body() const
{
  return body_;
}

void Drone::Draw(cv::Mat& image, geometric::Point t_vec, double m2px) const
{
  // Draw the LiDAR sensor
  lidar_.Draw(image, t_vec, m2px);

  // Draw the drone body (black rectangle)
  body_.Draw(image, cv::Scalar(0, 0, 0), t_vec, m2px);
}

lidar::Lidar Drone::Lidar() const
{
  return lidar_;
}

std::vector<double> Drone::LidarScan(
    const std::vector<geometric::Circle>& obstacles)
{
  return lidar_.Scan(obstacles);
}

geometric::Point Drone::Move(double dt, const geometric::Point& velocity)
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
  geometric::Point position_delta = velocity_ * dt;
  geometric::Point new_position = position_ + position_delta;

  // Update the LiDAR sensor position
  lidar_.UpdatePosition(new_position);

  // Update the drone position
  body_.UpdatePosition(new_position);
  position_ = new_position;

  return position_;
}

geometric::Point Drone::Position() const
{
  return position_;
}

void Drone::Reset(const geometric::Point& position)
{
  // Reset the drone position
  position_ = position;
  body_.UpdatePosition(position);

  // Reset the drone velocity
  velocity_ = geometric::Point(0, 0);

  // Reset the LiDAR sensor
  lidar_.Reset(position_);
}

}  // namespace drone
}  // namespace evs