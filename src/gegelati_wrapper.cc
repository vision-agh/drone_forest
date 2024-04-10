#include <drone_forest/gegelati_wrapper.h>

namespace evs
{
namespace drone_forest
{

const std::vector<geometric::Point> GegelatiWrapper::actions = {
    geometric::Point(0, 1), geometric::Point(-1, 0), geometric::Point(0, -1),
    geometric::Point(1, 0)};

GegelatiWrapper::GegelatiWrapper(
    double sim_step, std::tuple<double, double> xlim,
    std::tuple<double, double> ylim, int n_trees, double tree_min_radius,
    double tree_max_radius, int n_lidar_beams, double lidar_range,
    double min_tree_spare_distance, int max_spawn_attempts, double max_speed,
    double max_acceleration, int img_height, std::string window_name)
    : LearningEnvironment(actions.size()),
      drone_forest_(sim_step, xlim, ylim, n_trees, tree_min_radius,
                    tree_max_radius, n_lidar_beams, lidar_range,
                    min_tree_spare_distance, max_spawn_attempts, max_speed,
                    max_acceleration, img_height, window_name),
      xlim_(xlim),
      ylim_(ylim),
      is_collision_(false),
      is_success_(false)
{
  std::vector<double>& lidar_distances = drone_forest_.GetLidarDistances();
  for (double& distance : lidar_distances)
  {
    Data::PointerWrapper<double> distance_ptr;
    distance_ptr.setPointer(&distance);
    lidar_distances_.push_back(distance_ptr);
  }
  last_drone_position_ = drone_forest_.GetDronePosition();
}

std::vector<std::reference_wrapper<const Data::DataHandler>>
GegelatiWrapper::getDataSources()
{
  std::vector<std::reference_wrapper<const Data::DataHandler>> dataSources;
  for (Data::PointerWrapper<double>& distance : lidar_distances_)
  {
    dataSources.push_back(distance);
  }
  return dataSources;
}

void GegelatiWrapper::reset(size_t seed, Learn::LearningMode mode)
{
  drone_forest_.Reset();
  is_collision_ = false;
  is_success_ = false;
  last_drone_position_ = drone_forest_.GetDronePosition();
  accumulated_reward_ = 0;
}

void GegelatiWrapper::doAction(uint64_t actionID)
{
  drone_forest_.Step(actions[actionID]);

  // Collision check
  geometric::Point drone_position = drone_forest_.GetDronePosition();
  is_collision_ = drone_position.x() < std::get<0>(xlim_)
                  || drone_position.x() > std::get<1>(xlim_)
                  || drone_position.y() < std::get<0>(ylim_)
                  || drone_position.y() > std::get<1>(ylim_);

  if (!is_collision_)
  {
    for (double distance : drone_forest_.GetLidarDistances())
    {
      if (distance < kTreeCollisionDistance)
      {
        is_collision_ = true;
        break;
      }
    }
  }

  // Success check
  is_success_ = drone_position.y() >= std::get<1>(ylim_) - 2.0;

  // Reward calculation
  if (is_collision_)
  {
    accumulated_reward_ += kPenaltyTreeCollision;
  }
  else if (!is_success_)
  {
    // Center line reward
    accumulated_reward_ +=
        kPenaltyFarFromCenterLineCoeff * std::abs(drone_position.x());

    // Direction reward
    accumulated_reward_ += drone_position.y() > last_drone_position_.y()
                               ? kRewardGoodDirection
                               : kPenaltyWrongDirection;

    // Close to tree penalty
    for (double distance : drone_forest_.GetLidarDistances())
    {
      if (distance < kTreeSafeDistance)
      {
        accumulated_reward_ += kPenaltyCloseToTree;
      }
    }
  }

  last_drone_position_ = drone_position;
}

double GegelatiWrapper::getScore() const
{
  return accumulated_reward_;
}

bool GegelatiWrapper::isTerminal() const
{
  return is_collision_ || is_success_ || drone_forest_.GetTime() > kMaxSimTimeS;
}

}  // namespace drone_forest
}  // namespace evs