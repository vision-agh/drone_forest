#include <drone_forest/forest.h>

namespace evs
{
namespace forest
{

std::mt19937 Forest::gen_(std::random_device{}());

Forest::Forest(const std::tuple<double, double> x_limits,
               const std::tuple<double, double> y_limits, double y_limit_static,
               int num_trees, double min_radius, double max_radius,
               std::vector<geometric::Circle> exclusion_zones,
               double min_spare_distance, int max_spawn_attempts)
{
  // Generate uniform distribution for static trees
  std::uniform_real_distribution<double> x_dist(std::get<0>(x_limits),
                                                std::get<1>(x_limits));
  std::uniform_real_distribution<double> y_dist(std::get<0>(y_limits),
                                                y_limit_static);
  std::uniform_real_distribution<double> radius_dist(min_radius, max_radius);

  // Generate static trees
  double max_y = std::get<0>(y_limits);
  for (int idx_tree = 0; idx_tree < num_trees; idx_tree++)
  {
    for (int idx_attempt = 0; idx_attempt < max_spawn_attempts; idx_attempt++)
    {
      // Generate random tree
      Tree tree(geometric::Point(x_dist(gen_), y_dist(gen_)),
                radius_dist(gen_));

      // Check if tree is in exclusion zone
      bool in_exclusion_zone = false;
      for (const geometric::Circle& exclusion_zone : exclusion_zones)
      {
        if (tree.Trunk().Center().Distance(exclusion_zone.Center())
            < tree.Trunk().Radius() + exclusion_zone.Radius())
        {
          in_exclusion_zone = true;
          break;
        }
      }

      // Skip tree if in exclusion zone
      if (in_exclusion_zone)
      {
        continue;
      }

      // Check if minimum distance to other trees is satisfied
      bool min_distance_satisfied = true;
      for (const Tree& other_tree : trees_)
      {
        if (tree.Trunk().Center().Distance(other_tree.Trunk().Center())
            < tree.Trunk().Radius() + other_tree.Trunk().Radius()
                  + min_spare_distance)
        {
          min_distance_satisfied = false;
          break;
        }
      }

      // Skip tree if minimum distance is not satisfied
      if (!min_distance_satisfied)
      {
        continue;
      }

      // Add tree to forest
      trees_.push_back(Tree(tree));

      // Check if tree is the y-most tree
      double tree_max_y = tree.Trunk().Center().y() + tree.Trunk().Radius();
      if (tree_max_y > max_y)
      {
        max_y = tree_max_y;
      }
      break;
    }
  }

  // Generate moving trees
  if (y_limit_static < std::get<1>(y_limits))
  {
    GenerateMovingTrees(max_y, std::get<1>(y_limits), x_dist, radius_dist,
                        min_spare_distance);
  }
}

void Forest::Draw(cv::Mat& img, geometric::Point t_vec, double m2px) const
{
  for (const Tree& tree : trees_)
  {
    tree.Draw(img, t_vec, m2px);
  }
}

std::vector<geometric::Circle> Forest::GetObstacles() const
{
  std::vector<geometric::Circle> obstacles;
  for (const Tree& tree : trees_)
  {
    obstacles.push_back(tree.Trunk());
  }
  return obstacles;
}

int Forest::NumberOfTrees() const
{
  return trees_.size();
}

void Forest::UpdateMovingTreesPosition(double dt,
                                       std::tuple<double, double> x_limits)
{
  for (auto& [idx_tree, speed] : tree_speeds_)
  {
    // Calculate new x position
    double new_x = trees_[idx_tree].Trunk().Center().x() + speed * dt;

    // Check if tree is out of bounds
    if (new_x < std::get<0>(x_limits) || new_x > std::get<1>(x_limits))
    {
      // Change direction
      speed *= -1;

      // Bounce back the tree
      if (new_x < std::get<0>(x_limits))
      {
        double x_offset = std::get<0>(x_limits) - new_x;
        new_x = std::get<0>(x_limits) + x_offset;
      }
      else
      {
        double x_offset = new_x - std::get<1>(x_limits);
        new_x = std::get<1>(x_limits) - x_offset;
      }
    }

    // Update tree position
    trees_[idx_tree].UpdatePosition(
        geometric::Point(new_x, trees_[idx_tree].Trunk().Center().y()));
  }
}

std::vector<Tree> Forest::Trees() const
{
  return trees_;
}

void Forest::GenerateMovingTrees(
    double y_current_min, double y_max,
    std::uniform_real_distribution<double>& x_dist,
    std::uniform_real_distribution<double>& radius_dist,
    double min_spare_distance)
{
  // Generate distribution for moving trees
  std::uniform_real_distribution<double> moving_speed_dist(-5.0, 5.0);

  while (y_current_min < y_max)
  {
    // Generate tree
    double radius = radius_dist(gen_);
    Tree tree(geometric::Point(x_dist(gen_),
                               y_current_min + radius + min_spare_distance),
              radius);
    double speed = moving_speed_dist(gen_);

    // Add tree to forest
    trees_.push_back(tree);
    tree_speeds_[trees_.size() - 1] = speed;

    // Update y limit
    y_current_min += 2 * radius + min_spare_distance;
  }
}

}  // namespace forest
}  // namespace evs