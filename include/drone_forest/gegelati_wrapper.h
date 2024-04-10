#ifndef _DRONE_FOREST_GEGELATI_WRAPPER_H_
#define _DRONE_FOREST_GEGELATI_WRAPPER_H_

#include <drone_forest/drone_forest.h>
#include <gegelati.h>

#include <tuple>
#include <vector>

namespace evs
{
namespace drone_forest
{

const double kMaxSimTimeS = 120.0;

const double kTreeSafeDistance = 0.15;
const double kTreeCollisionDistance = 0.2;
const double kPenaltyCloseToTree = -0.25;
const double kPenaltyTreeCollision = -1.5;
const double kPenaltyFarFromCenterLineCoeff = -0.1;
const double kPenaltyWrongDirection = -3.0;
const double kRewardGoodDirection = 0.8;

/**
 * @brief Class representing a wrapper for the DroneForest class to be used with
 * the Gegelati library.
 *
 */
class GegelatiWrapper : public Learn::LearningEnvironment
{
 public:
  /**
   * @brief Actions that the drone can take.
   */
  static const std::vector<geometric::Point> actions;

  /**
   * @brief Environment instance.
   */
  DroneForest drone_forest_;

  /**
   * @brief LiDAR distances - observation space.
   */
  std::vector<Data::PointerWrapper<double>> lidar_distances_;

  /**
   * @brief Accumulated reward for the current episode.
   */
  double accumulated_reward_;

  /**
   * @brief Construct a new Gegelati Wrapper object
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
  GegelatiWrapper(double sim_step, std::tuple<double, double> xlim,
                  std::tuple<double, double> ylim, int n_trees,
                  double tree_min_radius, double tree_max_radius,
                  int n_lidar_beams, double lidar_range,
                  double min_tree_spare_distance, int max_spawn_attempts,
                  double max_speed, double max_acceleration,
                  int img_height = 800,
                  std::string window_name = "Drone Forest");

  virtual std::vector<std::reference_wrapper<const Data::DataHandler>>
  getDataSources() override;

  virtual void reset(size_t seed, Learn::LearningMode) override;

  virtual void doAction(uint64_t actionID) override;

  virtual double getScore() const override;

  virtual bool isTerminal() const override;

 private:
  std::tuple<double, double> xlim_;
  std::tuple<double, double> ylim_;

  bool is_collision_;
  bool is_success_;

  geometric::Point last_drone_position_;
};

}  // namespace drone_forest
}  // namespace evs

#endif  // _DRONE_FOREST_GEGELATI_WRAPPER_H_