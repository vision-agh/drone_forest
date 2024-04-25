#ifndef _DRONE_FOREST_DRONE_H_
#define _DRONE_FOREST_DRONE_H_

#include <drone_forest/geometric.h>
#include <drone_forest/lidar.h>

namespace evs
{
namespace drone
{

/**
 * @brief Simple drone dynamics model
 *
 */
class Drone
{
 public:
  /**
   * @brief Construct a new Drone object
   *
   * @param position Initial position of the drone
   * @param lidar_range Maximum range of the LiDAR sensor
   * @param lidar_n_beams Number of beams of the LiDAR sensor
   * @param max_speed Maximum speed of the drone
   * @param max_acceleration Maximum acceleration of the drone
   * @param drone_width_m Width of the drone in meters
   * @param drone_height_m Height of the drone in meters
   */
  Drone(const geometric::Point& position, double lidar_range, int lidar_n_beams,
        double max_speed = 1.0, double max_acceleration = 0.6,
        double drone_width_m = 0.1, double drone_height_m = 0.2)
      : position_(position),
        velocity_(0, 0),
        max_speed_(max_speed),
        max_acceleration_(max_acceleration),
        lidar_(position, lidar_range, lidar_n_beams),
        body_(
            position + geometric::Point(drone_width_m / 2, drone_height_m / 2),
            position - geometric::Point(drone_width_m / 2, drone_height_m / 2))
  {
  }

  /**
   * @brief Return the current position of the drone
   *
   * @return geometric::Point Current position of the drone
   */
  geometric::Point position() const
  {
    return position_;
  }

  /**
   * @brief Return the LiDAR sensor of the drone
   *
   * @return lidar::Lidar The LiDAR sensor
   */
  lidar::Lidar lidar() const
  {
    return lidar_;
  }

  /**
   * @brief Draw the drone on an image
   *
   * @param image Image to draw the drone on
   * @param t_vec Translation vector
   * @param m2px Meters to pixels conversion factor
   */
  void Draw(cv::Mat& image, geometric::Point t_vec, double m2px) const;

  /**
   * @brief Perform a LiDAR scan
   *
   * @param obstacles Obstacles to scan
   * @return std::vector<double> Distances to the obstacles measured by the
   * LiDAR sensor
   */
  std::vector<double> LidarScan(
      const std::vector<geometric::Circle>& obstacles);

  /**
   * @brief Move the drone
   *
   * @param dt Time step
   * @param velocity Desired velocity
   * @return geometric::Point New position of the drone
   */
  geometric::Point Move(double dt, const geometric::Point& velocity);

  /**
   * @brief Reset the drone state
   */
  void Reset(const geometric::Point& position);

 private:
  geometric::Point position_;
  geometric::Point velocity_;
  double max_speed_;
  double max_acceleration_;
  lidar::Lidar lidar_;
  geometric::Rectangle body_;
};

}  // namespace drone
}  // namespace evs

#endif  // _DRONE_FOREST_DRONE_H_