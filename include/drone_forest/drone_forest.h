#ifndef _DRONE_FOREST_SIMULATION_H_
#define _DRONE_FOREST_SIMULATION_H_

#include <drone_forest/drone.h>
#include <drone_forest/forest.h>

#include <opencv4/opencv2/opencv.hpp>
#include <string>
#include <tuple>
#include <vector>

namespace evs
{
namespace drone_forest
{

/**
 * @brief Class representing a simple simulation of a drone flying through a
 * forest.
 *
 * The simulation consists of a drone flying through a forest of trees. The
 * drone has a LiDAR sensor that can detect the trees in the forest. The drone
 * can move in the x-y plane with a given speed and acceleration. The forest is
 * generated randomly with a given number of trees and tree sizes.
 */
class DroneForest
{
 public:
  /**
   * @brief Construct a new DroneForest object
   *
   * @param sim_step Time step of the simulation (in seconds)
   * @param xlim x-axis limits of the simulation area (min, max in meters)
   * @param ylim y-axis limits of the simulation area (min, max in meters)
   * @param n_trees Number of trees in the forest
   * @param tree_min_radius Minimum radius of a tree
   * @param tree_max_radius Maximum radius of a tree
   * @param n_lidar_beams Number of beams of the LiDAR sensor
   * @param lidar_range Maximum range of the LiDAR sensor
   * @param min_tree_spare_distance Minimum distance between trees
   * @param max_spawn_attempts Maximum number of attempts to spawn a tree
   * @param max_speed Maximum speed of the drone
   * @param max_acceleration Maximum acceleration of the drone
   * @param img_height Height of the image to render
   * @param window_name Name of the window to render
   */
  DroneForest(double sim_step, std::tuple<double, double> xlim,
              std::tuple<double, double> ylim, int n_trees,
              double tree_min_radius, double tree_max_radius, int n_lidar_beams,
              double lidar_range, double min_tree_spare_distance,
              int max_spawn_attempts, double max_speed, double max_acceleration,
              int img_height = 800, std::string window_name = "Drone Forest");

  /**
   * @brief Get the Drone Position object
   *
   * @return geometric::Point& Drone position
   */
  geometric::Point& GetDronePosition()
  {
    return drone_position_;
  }

  /**
   * @brief Get the image of the simulation
   *
   * @note This function should be called after Render()
   *
   * @return cv::Mat& Image of the simulation
   */
  cv::Mat& GetImage()
  {
    return img_;
  }

  /**
   * @brief Get the distances detected by the LiDAR sensor
   *
   * @return std::vector<double>& Distances detected by the LiDAR sensor
   */
  std::vector<double>& GetLidarDistances();

  /**
   * @brief Get the time of the simulation
   *
   * @return double Time of the simulation (in seconds)
   */
  double GetTime() const;

  /**
   * @brief Render the simulation on the image
   */
  void Render();

  /**
   * @brief Reset the simulation to the initial state
   *
   * This method uses the internal drone Reset() method and creates a new
   * forest with the same parameters as the original one.
   */
  void Reset();

  /**
   * @brief Reset the simulation to the initial state with a given seed
   *
   * This is the overloaded version of the Reset() method that allows to set
   * the seed of the random number generator of the Forest object.
   *
   * @param seed Seed for the random number generator
   */
  void Reset(int seed);

  /**
   * @brief Perform a simulation step
   *
   * This method moves the drone according to the given velocity and performs a
   * LiDAR scan to detect the trees in the forest.
   *
   * @param velocity Velocity of the drone (in m/s)
   */
  void Step(geometric::Point velocity);

 private:
  const double sim_step_;
  double sim_time_;
  std::vector<double> lidar_distances_;

  drone::Drone drone_;
  geometric::Point drone_position_;

  forest::Forest forest_;
  const std::tuple<double, double> xlim_;
  const std::tuple<double, double> ylim_;
  const int n_trees_;
  const double tree_min_radius_;
  const double tree_max_radius_;
  const double min_tree_spare_distance_;
  const int max_spwan_attempts_;

  const std::string window_name_;
  cv::Mat img_;
  const int img_height_;
  int img_width_;
  const geometric::Point t_vec_;
  double m2px_;
};

}  // namespace drone_forest
}  // namespace evs

#endif  // _DRONE_FOREST_SIMULATION_H_