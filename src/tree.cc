#include <drone_forest/tree.h>

namespace evs
{
namespace forest
{

Tree::Tree() : trunk_(geometric::Point(0, 0), 0) {}

Tree::Tree(const geometric::Point& center, double radius)
    : trunk_(center, radius)
{
}

Tree::Tree(const geometric::Circle& trunk) : trunk_(trunk) {}

geometric::Circle Tree::Trunk() const
{
  return trunk_;
}

void Tree::Draw(cv::Mat& img, geometric::Point t_vec, double m2px) const
{
  // NOTE: Tree has a brown color
  // cv::Scalar color(139, 69, 19);
  cv::Scalar color(13, 63, 112);
  trunk_.Draw(img, color, t_vec, m2px);
}

void Tree::UpdatePosition(geometric::Point new_center)
{
  trunk_.UpdatePosition(new_center);
}

}  // namespace forest
}  // namespace evs