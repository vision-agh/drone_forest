#ifndef _DRONE_FOREST_GEGELATI_WRAPPER_H_
#define _DRONE_FOREST_GEGELATI_WRAPPER_H_

#include <drone_forest/drone_forest.h>
#include <gegelati.h>

#include <deque>
#include <filesystem>
#include <fstream>
#include <opencv4/opencv2/opencv.hpp>
#include <tuple>
#include <vector>

namespace fs = std::filesystem;
namespace evs
{
namespace drone_forest
{

// const double kTreeSafeDistance = 0.15;
const double kTreeCollisionDistance = 0.2;
// const double kRewardSuccess = 1.0;
// const double kPenaltyCloseToTree = -0.25;
// const double kPenaltyTreeCollision = -3.0;
// const double kPenaltyFarFromCenterLineCoeff = -0.05;
// const double kPenaltyWrongDirection = -3.0;
// const double kRewardGoodDirection = 0.8;

/**
 * @brief Class representing a wrapper for the DroneForest class to be used with
 * the Gegelati library.
 *
 */
class GegelatiWrapper : public Learn::LearningEnvironment
{
 public:
  /**
   * @brief Construct a new Gegelati Wrapper object
   *
   * @param actions Vector of predefined actions for the drone.
   * @param sim_step Time step of the simulation (in seconds)
   * @param xlim x-axis limits of the simulation area (min, max in meters)
   * @param ylim y-axis limits of the simulation area (min, max in meters)
   * @param y_static_limit y-limit of the static part of the forest
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
   * @param mode Learning mode in which the environment should be initialized
   */
  GegelatiWrapper(const std::vector<geometric::Point>& actions, double sim_step,
                  std::tuple<double, double> xlim,
                  std::tuple<double, double> ylim, double y_static_limit,
                  int n_trees, double tree_min_radius, double tree_max_radius,
                  int n_lidar_beams, double lidar_range,
                  double min_tree_spare_distance, int max_spawn_attempts,
                  double max_speed, double drone_width_m, double drone_height_m,
                  double max_acceleration, int img_height = 800,
                  std::string window_name = "Drone Forest",
                  Learn::LearningMode mode = Learn::LearningMode::TRAINING);

  /**
   * @brief Copy constructor for the GegelatiWrapper class.
   *
   * @param other Reference to the object to be copied.
   */
  GegelatiWrapper(const GegelatiWrapper& other);

  /**
   * @brief Get the data sources for the learning environment.
   *
   * This method is inherited from the LearningEnvironment interface.
   *
   * Data source in this learning environment is the LiDAR sensor. Therefore,
   * the method returns a vector of references to the distances measured by the
   * LiDAR sensor.
   *
   * @return std::vector<std::reference_wrapper<const Data::DataHandler>>
   */
  virtual std::vector<std::reference_wrapper<const Data::DataHandler>>
  getDataSources() override;

  /**
   * @brief Reset the learning environment.
   *
   * This method is inherited from the LearningEnvironment interface.
   *
   * The method resets the environment and sets the accumulated reward to zero.
   * During the reset, the drone is placed at the origin of the coordinate
   * system and lidar distances are set to the maximum range. The new forest is
   * generated from scratch.
   *
   * @param seed Integer for controlling randomness.
   * @param mode Learning mode in which environment should be reset.
   */
  virtual void reset(size_t seed, Learn::LearningMode mode) override;

  /**
   * @brief Execute an action in the learning environment.
   *
   * This method is inherited from the LearningEnvironment interface.
   *
   * The method executes the action in the environment. The action is a desired
   * velocity vector for the drone. Through this method, one of the predefined
   * actions (stored in internal vector) is selected and passed to the
   * simulation under the hood. After executing the action, the method updates
   * the internal state of the learning environment, i.e. accumulates the reward
   * and checks if the episode is terminal.
   *
   * @param actionID ID of the action to be executed.
   */
  virtual void doAction(uint64_t actionID) override;

  /**
   * @brief Return if the learning environment is copyable.
   *
   * This method is inherited from the LearningEnvironment interface.
   *
   * @return true Environment is copyable.
   * @return false Environment is not copyable.
   */
  virtual bool isCopyable() const override;

  virtual LearningEnvironment* clone() const override;

  /**
   * @brief Get the score for the current episode.
   *
   * This method is inherited from the LearningEnvironment interface.
   *
   * Simple getter for the accumulated score. It is calculated by accumulating
   * the rewards for each action executed during the episode (in doAction()).
   * The reward is calculated based on the following elements:
   * - collision with the tree,
   * - distance from the center line,
   * - distance from the trees,
   * - direction of the drone movement.
   *
   * @return double Score for the current episode.
   */
  virtual double getScore() const override;

  /**
   * @brief Check if drone is in a terminal state.
   *
   * This method is required by the LearningEnvironment interface.
   *
   * There are three possible scenarios for the terminal state:
   * - the drone collided with the tree or run out of the simulation area,
   * - the drone successfully flew through the forest,
   * - the simulation time exceeded the maximum time.
   *
   * @return true if the episode is terminal, false otherwise.
   */
  virtual bool isTerminal() const override;

  /**
   * @brief Render the current state of the learning environment.
   *
   * @return cv::Mat& The rendered image of the environment.
   */
  cv::Mat& Render();

  /**
   * @brief Get information about the positive outcome of the episode.
   *
   * @return true Episode was successful.
   * @return false Episode was not successful.
   */
  bool isSuccess() const;

 protected:
  /**
   * @brief Set the distances measured by the LiDAR sensor.
   *
   * @param distances Vector of distances measured by the LiDAR sensor.
   */
  void SetLidarDistances(const std::vector<double>& distances);

 private:
  // Rendering components
  cv::Mat render_;
  const int score_panel_height_ = 100;

  // Learning environment components
  const std::vector<geometric::Point> actions_;
  Data::PrimitiveTypeArray<double> lidar_distances_;
  DroneForest drone_forest_;
  double accumulated_reward_;
  double last_reward_;
  Learn::LearningMode mode_;

  // Internal state
  uint64_t last_action_id_;
  bool is_collision_;
  bool is_success_;
  geometric::Point last_drone_position_;
};

}  // namespace drone_forest
}  // namespace evs

#endif  // _DRONE_FOREST_GEGELATI_WRAPPER_H_