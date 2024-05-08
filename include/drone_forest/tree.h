#ifndef _DRONE_FOREST_TREE_H_
#define _DRONE_FOREST_TREE_H_

#include <drone_forest/geometric.h>

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

}  // namespace forest
}  // namespace evs

#endif  // _DRONE_FOREST_TREE_H_