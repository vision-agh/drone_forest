#include <drone_forest/forest.h>

namespace evs
{
namespace forest
{

Forest::Forest(const std::tuple<double, double> x_limits,
               const std::tuple<double, double> y_limits, int num_trees,
               double min_radius, double max_radius,
               std::vector<geometric::Circle> exclusion_zones,
               double min_spare_distance, int max_spawn_attempts)
{
  // Generate random number generator
  std::random_device rd;
  std::mt19937 gen(rd());

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
      geometric::Circle tree(geometric::Point(x_dist(gen), y_dist(gen)),
                             radius_dist(gen));

      // Check if tree is in exclusion zone
      bool in_exclusion_zone = false;
      for (const geometric::Circle& exclusion_zone : exclusion_zones)
      {
        if (tree.center().Distance(exclusion_zone.center())
            < tree.radius() + exclusion_zone.radius() + min_spare_distance)
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

      // Add tree to forest
      trees_.push_back(Tree(tree));
      break;
    }
  }
}

}  // namespace forest
}  // namespace evs