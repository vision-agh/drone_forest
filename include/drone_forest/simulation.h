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
namespace simulation
{

class Simulation
{
 public:
  Simulation(double sim_step, std::tuple<double, double> xlim,
             std::tuple<double, double> ylim, int n_trees,
             double tree_min_radius, double tree_max_radius, int n_lidar_beams,
             double lidar_range, double min_tree_spare_distance,
             int max_spawn_attempts, double max_speed, double max_acceleration,
             int img_height = 800, std::string window_name = "Drone Forest");

  cv::Mat GetImage()
  {
    return img_;
  }

  void Render();

  void Reset();

  void Reset(int seed);

  std::vector<double> Step(geometric::Point velocity);

 private:
  double sim_step_;

  drone::Drone drone_;

  forest::Forest forest_;
  std::tuple<double, double> xlim_;
  std::tuple<double, double> ylim_;
  int n_trees_;
  double tree_min_radius_;
  double tree_max_radius_;
  double min_tree_spare_distance_;
  int max_spwan_attempts_;

  std::string window_name_;
  cv::Mat img_;
  int img_height_;
  int img_width_;
  geometric::Point t_vec_;
  double m2px_;
};

}  // namespace simulation
}  // namespace evs

#endif  // _DRONE_FOREST_SIMULATION_H_