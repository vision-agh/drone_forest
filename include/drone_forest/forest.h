#ifndef _DRONE_FOREST_FOREST_H_
#define _DRONE_FOREST_FOREST_H_

#include <drone_forest/geometric.h>

#include <opencv4/opencv2/opencv.hpp>
#include <random>
#include <tuple>
#include <vector>

namespace evs
{
namespace forest
{

/**
 * @brief A simple 2D tree class
 *
 * The tree is represented by a trunk, which is a circle.
 */
class Tree
{
 public:
  /**
   * @brief Construct a default Tree object
   *
   */
  Tree() : trunk_(geometric::Point(0, 0), 0) {}

  /**
   * @brief Construct a new Tree object
   *
   * @param center Center of the tree's trunk
   * @param radius Radius of the tree's trunk
   */
  Tree(const geometric::Point& center, double radius) : trunk_(center, radius)
  {
  }

  /**
   * @brief Construct a new Tree object
   *
   * @param trunk Circle representing the tree's trunk
   */
  Tree(const geometric::Circle& trunk) : trunk_(trunk) {}

  /**
   * @brief Get the trunk of the tree
   *
   * @return Circle Trunk of the tree
   */
  geometric::Circle trunk() const
  {
    return trunk_;
  }

  /**
   * @brief Get the center of the tree
   *
   * @return Point Center of the tree
   */
  geometric::Point center() const
  {
    return trunk_.center();
  }

  /**
   * @brief Get the radius of the tree
   *
   * @return double Radius of the tree
   */
  double radius() const
  {
    return trunk_.radius();
  }

  /**
   * @brief Draw the tree on an image
   *
   * @param img Image to draw the tree on
   * @param t_vec Translation vector
   * @param m2px Meters to pixels conversion factor
   */
  void Draw(cv::Mat& img, geometric::Point t_vec, double m2px) const
  {
    // NOTE: Tree has a brown color
    trunk_.Draw(img, cv::Scalar(19, 69, 139), t_vec, m2px);
  }

 private:
  geometric::Circle trunk_;
};

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
   * @brief Get the trees in the forest
   *
   * @return std::vector<Tree> Trees in the forest
   */
  std::vector<Tree> trees() const
  {
    return trees_;
  }

  /**
   * @brief Get the number of trees in the forest
   *
   * @return int Number of trees in the forest
   */
  int size() const
  {
    return trees_.size();
  }

  /**
   * @brief Draw the forest on an image
   *
   * @param img Image to draw the forest on
   * @param t_vec Translation vector
   * @param m2px Meters to pixels conversion factor
   */
  void Draw(cv::Mat& img, geometric::Point t_vec, double m2px) const;

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