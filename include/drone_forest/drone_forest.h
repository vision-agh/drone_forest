#ifndef _DRONE_FOREST_SIMULATION_H_
#define _DRONE_FOREST_SIMULATION_H_

#include <drone_forest/drone.h>
#include <drone_forest/forest.h>
#include <drone_forest/geometric/line.h>
#include <drone_forest/geometric/point.h>

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
   * @param y_static_limit y-limit of the static part of the forest
   * @param goal_y y-coordinate of the goal position
   * @param n_trees Number of trees in the forest
   * @param tree_min_radius Minimum radius of a tree
   * @param tree_max_radius Maximum radius of a tree
   * @param n_lidar_beams Number of beams of the LiDAR sensor
   * @param lidar_range Maximum range of the LiDAR sensor
   * @param min_tree_spare_distance Minimum distance between trees
   * @param max_spawn_attempts Maximum number of attempts to spawn a tree
   * @param max_speed Maximum speed of the drone
   * @param max_acceleration Maximum acceleration of the drone
   * @param drone_width_m Width of the drone in meters
   * @param drone_height_m Height of the drone in meters
   * @param img_height Height of the image to render
   * @param window_name Name of the window to render
   */
  DroneForest(double sim_step, std::tuple<double, double> xlim,
              std::tuple<double, double> ylim, double y_static_limit,
              double goal_y, int n_trees, double tree_min_radius,
              double tree_max_radius, int n_lidar_beams, double lidar_range,
              double min_tree_spare_distance, int max_spawn_attempts,
              double max_speed, double max_acceleration, double drone_width_m,
              double drone_height_m, int img_height = 800,
              std::string window_name = "Drone Forest");

  /**
   * @brief Check if the drone is colliding with a tree
   *
   * @return true The drone is colliding with a tree
   * @return false The drone is not colliding with a tree
   */
  bool CheckCollision() const;

  /**
   * @brief Check if the goal has been reached
   *
   * @return true The goal has been reached
   * @return false The goal has not been reached
   */
  bool CheckGoalReached() const;

  /**
   * @brief Calculate the distance to the goal
   *
   * @return double Distance to the goal (in meters)
   */
  double DistanceToGoal() const;

  /**
   * @brief Get the maximum speed of the drone
   *
   * @return double Maximum speed of the drone (in m/s)
   */
  double GetDroneMaxSpeed() const;

  /**
   * @brief Get the Drone Position object
   *
   * @return geometric::Point& Drone position
   */
  geometric::Point GetDronePosition() const;

  /**
   * @brief Get the Drone Position As Vector object
   *
   * This method is compatible with the Python bindings.
   *
   * @return std::vector<double> Drone position as a vector
   */
  std::vector<double> GetDronePositionAsVector() const;

  /**
   * @brief Get the image of the simulation
   *
   * @note This function should be called after Render()
   *
   * @return cv::Mat& Image of the simulation
   */
  const cv::Mat& GetImage() const;

  /**
   * @brief Get the Image As Vector object
   *
   * This method is compatible with the Python bindings. Solution based on:
   * https://stackoverflow.com/questions/26681713
   *
   * @return std::vector<uchar> Image of the simulation as a vector
   */
  std::vector<uchar> GetImageAsVector() const;

  /**
   * @brief Get the size of the image
   *
   * @return std::tuple<int, int> Size of the image (height, width)
   */
  std::tuple<int, int> GetImageSize() const;

  /**
   * @brief Get the distances detected by the LiDAR sensor
   *
   * @return std::vector<double>& Distances detected by the LiDAR sensor
   */
  std::vector<double>& GetLidarDistances();

  /**
   * @brief Get the distances detected by the LiDAR sensor as a vector
   *
   * This method is compatible with the Python bindings.
   *
   * @return std::vector<double> Distances detected by the LiDAR sensor as a
   * vector
   */
  std::vector<double> GetLidarDistancesAsVector() const;

  /**
   * @brief Get the limits of the x-axis
   *
   * @return std::tuple<double, double> Limits of the x-axis (min, max)
   */
  std::tuple<double, double> GetLimitsX() const;

  /**
   * @brief Get the limits of the y-axis
   *
   * @return std::tuple<double, double> Limits of the y-axis (min, max)
   */
  std::tuple<double, double> GetLimitsY() const;

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

  /**
   * @brief Perform a simulation step
   *
   * This method moves the drone according to the given velocity and performs a
   * LiDAR scan to detect the trees in the forest.
   *
   * This method is compatible with the Python bindings.
   *
   * @param velocity Velocity of the drone (in m/s)
   */
  void StepVelocityVector(std::vector<double> velocity);

 private:
  const double sim_step_;
  double sim_time_;
  std::vector<double> lidar_distances_;

  drone::Drone drone_;
  geometric::Point drone_position_;

  forest::Forest forest_;
  const std::tuple<double, double> xlim_;
  const std::tuple<double, double> ylim_;
  double y_static_limit_;
  double goal_y_;
  const int n_trees_;
  const double tree_min_radius_;
  const double tree_max_radius_;
  const double min_tree_spare_distance_;
  const int max_spawn_attempts_;

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