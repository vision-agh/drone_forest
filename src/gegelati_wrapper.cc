#include <drone_forest/gegelati_wrapper.h>

namespace evs
{
namespace drone_forest
{

GegelatiWrapper::GegelatiWrapper(
    const std::vector<geometric::Point>& actions, double sim_step,
    std::tuple<double, double> xlim, std::tuple<double, double> ylim,
    double y_limit_static, int n_trees, double tree_min_radius,
    double tree_max_radius, int n_lidar_beams, double lidar_range,
    double min_tree_spare_distance, int max_spawn_attempts, double max_speed,
    double max_acceleration, double drone_width_m, double drone_height_m,
    int img_height, std::string window_name, Learn::LearningMode mode)
    : LearningEnvironment(actions.size()),
      actions_(actions),
      lidar_distances_(n_lidar_beams),
      drone_forest_(sim_step, xlim, ylim, y_limit_static,
                    std::get<1>(ylim) - 2.0, n_trees, tree_min_radius,
                    tree_max_radius, n_lidar_beams, lidar_range,
                    min_tree_spare_distance, max_spawn_attempts, max_speed,
                    max_acceleration, drone_width_m, drone_height_m, img_height,
                    window_name),
      mode_(mode),
      accumulated_reward_(0),
      last_reward_(0),
      last_action_id_(0),
      is_collision_(false),
      is_success_(false)
{
  // Render is 100 px higher than the image to display the score
  SetLidarDistances(drone_forest_.GetLidarDistances());
  render_ =
      cv::Mat(drone_forest_.GetImage().rows + score_panel_height_,
              drone_forest_.GetImage().cols, CV_8UC3, cv::Scalar(0, 0, 0));
  last_drone_position_ = drone_forest_.GetDronePosition();
}

GegelatiWrapper::GegelatiWrapper(const GegelatiWrapper& other)
    : LearningEnvironment(actions_.size()),
      actions_(other.actions_),
      lidar_distances_(other.lidar_distances_),
      drone_forest_(other.drone_forest_),
      accumulated_reward_(other.accumulated_reward_),
      last_reward_(other.last_reward_),
      mode_(other.mode_),
      last_action_id_(other.last_action_id_),
      is_collision_(other.is_collision_),
      is_success_(other.is_success_),
      last_drone_position_(other.last_drone_position_)
{
  // Deep copy of the render image
  render_ = other.render_.clone();
}

std::vector<std::reference_wrapper<const Data::DataHandler>>
GegelatiWrapper::getDataSources()
{
  auto dataSources =
      std::vector<std::reference_wrapper<const Data::DataHandler>>();
  dataSources.push_back(lidar_distances_);
  return dataSources;
}

void GegelatiWrapper::reset(size_t seed, Learn::LearningMode mode)
{
  drone_forest_.Reset(seed);
  SetLidarDistances(drone_forest_.GetLidarDistances());
  is_collision_ = false;
  is_success_ = false;
  last_drone_position_ = drone_forest_.GetDronePosition();
  accumulated_reward_ = 0;
  last_reward_ = 0;
  last_action_id_ = 0;
  mode_ = mode;
}

void GegelatiWrapper::doAction(uint64_t actionID)
{
  // One step of the simulation
  drone_forest_.Step(actions_[actionID] * drone_forest_.GetDroneMaxSpeed());
  last_action_id_ = actionID;

  // Update observation
  SetLidarDistances(drone_forest_.GetLidarDistances());

  // Collision check
  is_collision_ = drone_forest_.CheckCollision();

  // Success check
  is_success_ = drone_forest_.CheckGoalReached();

  // Reward calculation
  // // Quite good configuration
  // if (is_collision_)
  // {
  //   last_reward_ = -5.0;
  // }
  // else if (last_drone_position_.y() < drone_position.y())
  // {
  //   last_reward_ = 1.0;
  // }
  // else
  // {
  //   last_reward_ = -2.0;
  // }

  // if (is_collision_)
  // {
  //   last_reward_ = -10.0;
  // }
  // else if (is_success_)
  // {
  //   last_reward_ = 50.0;
  // }
  // else if (last_drone_position_.y() < drone_position.y())
  // {
  //   last_reward_ = 1.0;
  // }
  // else if (last_drone_position_.y() > drone_position.y())
  // {
  //   last_reward_ = -2.0;
  // }
  // else
  // {
  //   last_reward_ = -1.0;  // Was: -1.0;
  // }

  // if (is_collision_)
  // {
  //   last_reward_ = -10.0;
  // }
  // else if (is_success_)
  // {
  //   last_reward_ = 50.0;
  // }
  // else
  // {
  //   double y_diff = drone_position.y() - last_drone_position_.y();
  //   if (y_diff > 0)
  //   {
  //     last_reward_ = 1.0 * y_diff;
  //   }
  //   else if (y_diff < 0)
  //   {
  //     last_reward_ = -5.0 * y_diff;
  //   }
  //   else
  //   {
  //     last_reward_ = 0.0;
  //   }
  // }

  // last_reward_ = -distance_to_goal;

  // last_reward_ = 0.0;

  // if (is_success_)
  // {
  //   last_reward_ += kRewardSuccess;
  // }
  // else if (is_collision_)
  // {
  //   last_reward_ += kPenaltyTreeCollision;
  // }
  // else
  // {
  //   // Center line reward
  //   last_reward_ +=
  //       kPenaltyFarFromCenterLineCoeff * std::abs(drone_position.x());

  //   // Direction reward
  //   last_reward_ += drone_position.y() > last_drone_position_.y()
  //                       ? kRewardGoodDirection
  //                       : kPenaltyWrongDirection;

  //   // Close to tree penalty
  //   for (double distance : drone_forest_.GetLidarDistances())
  //   {
  //     if (distance < kTreeSafeDistance)
  //     {
  //       last_reward_ += kPenaltyCloseToTree;
  //     }
  //   }
  // }

  if (mode_ != Learn::LearningMode::VALIDATION)
  {
    if (is_collision_)
    {
      accumulated_reward_ = -25.0;
    }
    else if (is_success_)
    {
      accumulated_reward_ = 100.0;
    }
    else
    {
      accumulated_reward_ = -drone_forest_.DistanceToGoal();
    }
  }
  else
  {
    accumulated_reward_ = drone_forest_.GetDronePosition().y();
  }

  // accumulated_reward_ += last_reward_;
  // last_drone_position_ = drone_forest_.GetDronePosition();
}

bool GegelatiWrapper::isCopyable() const
{
  return true;
}

Learn::LearningEnvironment* GegelatiWrapper::clone() const
{
  return new GegelatiWrapper(*this);
}

double GegelatiWrapper::getScore() const
{
  // if (is_collision_)
  // {
  //   return accumulated_reward_;
  // }
  // else if (is_success_)
  // {
  //   return 10.0 / drone_forest_.GetTime();
  // }
  // else
  // {
  //   return accumulated_reward_ / drone_forest_.GetTime();
  // }
  // if (isTerminal())
  // {
  //   return accumulated_reward_;
  // }
  // else
  // {
  //   return last_reward_;
  // }
  // return last_reward_;
  return accumulated_reward_;
}

bool GegelatiWrapper::isTerminal() const
{
  return is_collision_ || is_success_;
}

cv::Mat& GegelatiWrapper::Render()
{
  drone_forest_.Render();
  const cv::Mat& img = drone_forest_.GetImage();
  cv::Mat score_panel =
      cv::Mat(score_panel_height_, img.cols, CV_8UC3, cv::Scalar(0, 0, 0));
  cv::putText(score_panel, "Score: " + std::to_string(getScore()),
              cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1,
              cv::Scalar(255, 255, 255), 2);
  cv::putText(score_panel,
              "Time: " + std::to_string(drone_forest_.GetTime()) + " s",
              cv::Point(10, 70), cv::FONT_HERSHEY_SIMPLEX, 1,
              cv::Scalar(255, 255, 255), 2);
  cv::putText(score_panel, "Action: ", cv::Point(img.cols - 130, 30),
              cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
  cv::putText(score_panel, std::to_string(last_action_id_),
              cv::Point(img.cols - 100, 70), cv::FONT_HERSHEY_SIMPLEX, 1,
              cv::Scalar(255, 255, 255), 2);
  cv::vconcat(img, score_panel, render_);

  return render_;
}

bool GegelatiWrapper::isSuccess() const
{
  return is_success_;
}

geometric::Point GegelatiWrapper::GetDronePosition() const
{
  return drone_forest_.GetDronePosition();
}

void GegelatiWrapper::SetLidarDistances(const std::vector<double>& distances)
{
  double lidar_max_range = drone_forest_.GetLidarRange();
  for (size_t i = 0; i < distances.size(); i++)
  {
    lidar_distances_.setDataAt(typeid(double), i,
                               distances[i] / lidar_max_range);
  }
}

}  // namespace drone_forest
}  // namespace evs