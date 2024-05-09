#include <drone_forest/drone_forest.h>

#include <iostream>

namespace evs
{
namespace drone_forest
{

DroneForest::DroneForest(double sim_step, std::tuple<double, double> xlim,
                         std::tuple<double, double> ylim, double y_static_limit,
                         double goal_y, int n_trees, double tree_min_radius,
                         double tree_max_radius, int n_lidar_beams,
                         double lidar_range, double min_tree_spare_distance,
                         int max_spawn_attempts, double max_speed,
                         double max_acceleration, double drone_width_m,
                         double drone_height_m, int img_height,
                         std::string window_name)
    : sim_step_(sim_step),
      sim_time_(0.0),
      lidar_distances_(n_lidar_beams, lidar_range),
      drone_(geometric::Point(0, 0), lidar_range, n_lidar_beams, max_speed,
             max_acceleration, drone_width_m, drone_height_m),
      drone_position_(geometric::Point(0, 0)),
      forest_(xlim, ylim, y_static_limit, n_trees, tree_min_radius,
              tree_max_radius, {geometric::Circle(geometric::Point(0, 0), 1.0)},
              min_tree_spare_distance, max_spawn_attempts),
      xlim_(xlim),
      ylim_(ylim),
      y_static_limit_(y_static_limit),
      goal_y_(goal_y),
      n_trees_(n_trees),
      tree_min_radius_(tree_min_radius),
      tree_max_radius_(tree_max_radius),
      min_tree_spare_distance_(min_tree_spare_distance),
      max_spawn_attempts_(max_spawn_attempts),
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

bool DroneForest::CheckCollision() const
{
  // Check collision with boundaries
  if (drone_position_.x() < std::get<0>(xlim_)
      || drone_position_.x() > std::get<1>(xlim_)
      || drone_position_.y() < std::get<0>(ylim_)
      || drone_position_.y() > std::get<1>(ylim_))
  {
    return true;
  }

  // Check collision with trees
  for (const geometric::Circle& circ : forest_.GetObstacles())
  {
    if (drone_.Body().CheckCircleIntersection(circ))
    {
      return true;
    }
  }

  return false;
}

bool DroneForest::CheckGoalReached() const
{
  return drone_position_.y() >= goal_y_;
}

double DroneForest::DistanceToGoal() const
{
  return std::abs(goal_y_ - drone_position_.y());
}

double DroneForest::GetDroneMaxSpeed() const
{
  return drone_.GetMaxSpeed();
}

geometric::Point DroneForest::GetDronePosition() const
{
  return drone_position_;
}

std::vector<double> DroneForest::GetDronePositionAsVector() const
{
  std::vector<double> drone_position = {drone_position_.x(),
                                        drone_position_.y()};
  return drone_position;
}

const cv::Mat& DroneForest::GetImage() const
{
  return img_;
}

std::vector<uchar> DroneForest::GetImageAsVector() const
{
  std::vector<uchar> img_data;

  // Clone the image to ensure data continuity
  cv::Mat img = img_.clone();

  img_data.assign(img.data, img.data + img.total() * img.channels());
  return img_data;
}

std::tuple<int, int> DroneForest::GetImageSize() const
{
  return std::make_tuple(img_height_, img_width_);
}

std::vector<double>& DroneForest::GetLidarDistances()
{
  return lidar_distances_;
}

std::vector<double> DroneForest::GetLidarDistancesAsVector() const
{
  return lidar_distances_;
}

std::tuple<double, double> DroneForest::GetLimitsX() const
{
  return xlim_;
}

std::tuple<double, double> DroneForest::GetLimitsY() const
{
  return ylim_;
}

double DroneForest::GetTime() const
{
  return sim_time_;
}

void DroneForest::Render()
{
  // Create an image and fill it with green
  img_ = cv::Mat(img_height_, img_width_, CV_8UC3, cv::Scalar(0, 255, 0));

  // Draw the forest
  forest_.Draw(img_, t_vec_, m2px_);

  // Draw the drone
  drone_.Draw(img_, t_vec_, m2px_);

  // Draw the goal as a red line
  geometric::Point goal_left_end(std::get<0>(xlim_), goal_y_);
  geometric::Point goal_right_end(std::get<1>(xlim_), goal_y_);
  geometric::Line goal_line(goal_left_end, goal_right_end);
  goal_line.Draw(img_, cv::Scalar(0, 0, 255), t_vec_, m2px_);
}

void DroneForest::Reset()
{
  geometric::Point drone_start_position(0, 0);
  drone_.Reset(drone_start_position);
  drone_position_ = drone_start_position;
  forest_ = forest::Forest(xlim_, ylim_, y_static_limit_, n_trees_,
                           tree_min_radius_, tree_max_radius_,
                           {geometric::Circle(geometric::Point(0, 0), 1.0)},
                           min_tree_spare_distance_, max_spawn_attempts_);
  sim_time_ = 0.0;

  // NOTE: It is important to keep lidar_distances_ intact in terms of memory
  // allocation! Actually, now maybe it is not that important?
  std::vector<double> tmp_distances = drone_.LidarScan(forest_.GetObstacles());
  for (size_t i = 0; i < tmp_distances.size(); i++)
  {
    lidar_distances_[i] = tmp_distances[i];
  }
}

void DroneForest::Reset(int seed)
{
  forest::Forest::SetSeed(seed);
  Reset();
}

void DroneForest::Step(geometric::Point velocity)
{
  // Move the drone
  drone_position_ = drone_.Move(sim_step_, velocity);

  // Move the trees
  forest_.UpdateMovingTreesPosition(sim_step_, xlim_);

  // Update simulation time
  sim_time_ += sim_step_;

  // NOTE: It is important to keep lidar_distances_ intact in terms of memory
  // allocation! Actually, now maybe it is not that important?
  std::vector<double> tmp_distances = drone_.LidarScan(forest_.GetObstacles());
  for (size_t i = 0; i < tmp_distances.size(); i++)
  {
    lidar_distances_[i] = tmp_distances[i];
  }
}

void DroneForest::StepVelocityVector(std::vector<double> velocity)
{
  Step(geometric::Point(velocity[0], velocity[1]));
}

}  // namespace drone_forest
}  // namespace evs