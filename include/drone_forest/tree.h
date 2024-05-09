#ifndef _DRONE_FOREST_TREE_H_
#define _DRONE_FOREST_TREE_H_

#include <drone_forest/geometric/circle.h>
#include <drone_forest/geometric/point.h>

#include <opencv4/opencv2/opencv.hpp>

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
  Tree();

  /**
   * @brief Construct a new Tree object
   *
   * @param center Center of the tree's trunk
   * @param radius Radius of the tree's trunk
   */
  Tree(const geometric::Point& center, double radius);
  /**
   * @brief Construct a new Tree object
   *
   * @param trunk Circle representing the tree's trunk
   */
  Tree(const geometric::Circle& trunk);

  /**
   * @brief Get the trunk of the tree
   *
   * @return Circle Trunk of the tree
   */
  geometric::Circle Trunk() const;

  /**
   * @brief Draw the tree on an image
   *
   * @param img Image to draw the tree on
   * @param t_vec Translation vector
   * @param m2px Meters to pixels conversion factor
   */
  void Draw(cv::Mat& img, geometric::Point t_vec, double m2px) const;

 private:
  geometric::Circle trunk_;
};

}  // namespace forest
}  // namespace evs

#endif  // _DRONE_FOREST_TREE_H_