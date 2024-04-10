#include <drone_forest/drone_forest.h>

#include <iostream>

namespace evs
{
namespace drone_forest
{

DroneForest::DroneForest(double sim_step, std::tuple<double, double> xlim,
                         std::tuple<double, double> ylim, int n_trees,
                         double tree_min_radius, double tree_max_radius,
                         int n_lidar_beams, double lidar_range,
                         double min_tree_spare_distance, int max_spawn_attempts,
                         double max_speed, double max_acceleration,
                         int img_height, std::string window_name)
    : sim_step_(sim_step),
      drone_(geometric::Point(0, 0), lidar_range, n_lidar_beams, max_speed,
             max_acceleration),
      forest_(xlim, ylim, n_trees, tree_min_radius, tree_max_radius,
              {geometric::Circle(geometric::Point(0, 0), 1.0)},
              min_tree_spare_distance, max_spawn_attempts),
      xlim_(xlim),
      ylim_(ylim),
      n_trees_(n_trees),
      tree_min_radius_(tree_min_radius),
      tree_max_radius_(tree_max_radius),
      min_tree_spare_distance_(min_tree_spare_distance),
      max_spwan_attempts_(max_spawn_attempts),
      window_name_(window_name),
      img_height_(img_height),
      t_vec_(-std::get<0>(xlim), -std::get<0>(ylim))
{
  double y_range = std::get<1>(ylim) - std::get<0>(ylim);
  double x_range = std::get<1>(xlim) - std::get<0>(xlim);

  m2px_ = img_height_ / y_range;
  img_width_ = int(x_range * m2px_);
  img_ = cv::Mat(img_height_, img_width_, CV_8UC3, cv::Scalar(0, 255, 0));
}

void DroneForest::Render()
{
  // Create an image and fill it with green
  img_ = cv::Mat(img_height_, img_width_, CV_8UC3, cv::Scalar(0, 255, 0));

  // Draw the forest
  forest_.Draw(img_, t_vec_, m2px_);

  // Draw the drone
  drone_.Draw(img_, t_vec_, m2px_);
}

void DroneForest::Reset()
{
  drone_.Reset(geometric::Point(0, 0));
  forest_ =
      forest::Forest(xlim_, ylim_, n_trees_, tree_min_radius_, tree_max_radius_,
                     {geometric::Circle(geometric::Point(0, 0), 1.0)},
                     min_tree_spare_distance_, max_spwan_attempts_);
}

void DroneForest::Reset(int seed)
{
  forest::Forest::SetSeed(seed);
  Reset();
}

std::vector<double> DroneForest::Step(geometric::Point velocity)
{
  // Move the drone
  drone_.Move(sim_step_, velocity);

  // Get the lidar readings
  std::vector<double> lidar_readings = drone_.LidarScan(forest_.GetObstacles());

  return lidar_readings;
}

}  // namespace drone_forest
}  // namespace evs