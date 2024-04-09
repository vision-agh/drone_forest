#ifndef _DRONE_FOREST_LIDAR_H_
#define _DRONE_FOREST_LIDAR_H_

#include <drone_forest/geometric.h>

#include <cmath>
#include <vector>

namespace evs
{
namespace lidar
{

/**
 * @brief Class representing a simple 2D LiDAR sensor
 *
 */
class Lidar
{
 public:
  /**
   * @brief Construct a new Lidar object
   *
   * @param position Position of the LiDAR sensor
   * @param range Maximum range of the LiDAR sensor
   * @param n_beams Number of beams emitted by the LiDAR sensor
   */
  Lidar(const geometric::Point& position, double range, int n_beams);

  /**
   * @brief Return the position of the LiDAR sensor
   *
   * @return geometric::Point Position of the LiDAR sensor
   */
  geometric::Point position() const
  {
    return position_;
  }

  /**
   * @brief Return the range of the LiDAR sensor
   *
   * @return double The range of the LiDAR sensor
   */
  double range() const
  {
    return range_;
  }

  /**
   * @brief Return the angles of the beams emitted by the LiDAR sensor
   *
   * @return std::vector<double> Angles of the beams
   */
  std::vector<double> angles() const
  {
    return beam_angles_;
  }

  /**
   * @brief Return the beams emitted by the LiDAR sensor
   *
   * @return std::vector<geometric::Line> Beams emitted by the LiDAR sensor
   */
  std::vector<geometric::Line> beams() const
  {
    return beams_;
  }

  /**
   * @brief Draw the LiDAR sensor beams on an image
   *
   * @param image Image to draw the beams on
   * @param t_vec Translation vector to apply to the beams
   * @param m2px Meter to pixel conversion factor
   */
  void Draw(cv::Mat& image, geometric::Point t_vec, double m2px) const;

  /**
   * @brief Perform a LiDAR scan
   *
   * @param obstacles Vector of obstacles to scan
   * @return std::vector<double> Vector of distances to the obstacles
   */
  std::vector<double> Scan(const std::vector<geometric::Circle>& obstacles);

  /**
   * @brief Update the position of the LiDAR sensor
   *
   * @param new_position New position of the LiDAR sensor
   * @return geometric::Point Old position of the LiDAR sensor
   */
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