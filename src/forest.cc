#include <drone_forest/forest.h>

namespace evs
{
namespace forest
{

std::mt19937 Forest::gen_(std::random_device{}());

Forest::Forest(const std::tuple<double, double> x_limits,
               const std::tuple<double, double> y_limits, int num_trees,
               double min_radius, double max_radius,
               std::vector<geometric::Circle> exclusion_zones,
               double min_spare_distance, int max_spawn_attempts)
{
  // Generate uniform distribution for x and y
  std::uniform_real_distribution<double> x_dist(std::get<0>(x_limits),
                                                std::get<1>(x_limits));
  std::uniform_real_distribution<double> y_dist(std::get<0>(y_limits),
                                                std::get<1>(y_limits));
  std::uniform_real_distribution<double> radius_dist(min_radius, max_radius);

  // Generate trees
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
      break;
    }
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

std::vector<Tree> Forest::Trees() const
{
  return trees_;
}

}  // namespace forest
}  // namespace evs