#ifndef _DRONE_FOREST_FOREST_H_
#define _DRONE_FOREST_FOREST_H_

#include <drone_forest/geometric/circle.h>
#include <drone_forest/tree.h>

#include <opencv4/opencv2/opencv.hpp>
#include <random>
#include <tuple>
#include <vector>

namespace evs
{
namespace forest
{

class Forest
{
 public:
  /**
   * @brief Construct a new Forest object
   *
   * @param x_limits X-axis limits of the forest
   * @param y_limits Y-axis limits of the forest
   * @param num_trees Number of trees in the forest
   * @param min_radius Minimum radius of the trees
   * @param max_radius Maximum radius of the trees
   * @param exclusion_zones Exclusion zones for the trees
   * @param min_spare_distance Minimum distance between trees
   * @param max_spawn_attempts Maximum number of attempts to spawn a tree
   */
  Forest(const std::tuple<double, double> x_limits,
         const std::tuple<double, double> y_limits, int num_trees,
         double min_radius, double max_radius,
         std::vector<geometric::Circle> exclusion_zones,
         double min_spare_distance, int max_spawn_attempts = 50);

  /**
   * @brief Draw the forest on an image
   *
   * @param img Image to draw the forest on
   * @param t_vec Translation vector
   * @param m2px Meters to pixels conversion factor
   */
  void Draw(cv::Mat& img, geometric::Point t_vec, double m2px) const;

  /**
   * @brief Get the Obstacles in the forest
   *
   * @return std::vector<geometric::Circle> Obstacles in the forest
   */
  std::vector<geometric::Circle> GetObstacles() const;

  /**
   * @brief Get the number of trees in the forest
   *
   * @return int Number of trees in the forest
   */
  int NumberOfTrees() const;

  /**
   * @brief Get the trees in the forest
   *
   * @return std::vector<Tree> Trees in the forest
   */
  std::vector<Tree> Trees() const;

  /**
   * @brief Set the seed of the random number generator
   *
   * @param seed Seed for the random number generator
   */
  static void SetSeed(int seed)
  {
    gen_.seed(seed);
  }

 private:
  static std::mt19937 gen_;

  std::vector<Tree> trees_;
};

}  // namespace forest
}  // namespace evs

#endif  // _DRONE_FOREST_FOREST_H_