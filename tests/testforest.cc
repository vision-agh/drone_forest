#include <drone_forest/forest.h>
#include <gtest/gtest.h>

#include <cmath>
// #include <opencv4/opencv2/opencv.hpp>

TEST(ForestTest, TreeDefaultConstructor)
{
  evs::forest::Tree tree;
  evs::geometric::Circle trunk = tree.trunk();
  EXPECT_EQ(trunk.center().x(), 0);
  EXPECT_EQ(trunk.center().y(), 0);
  EXPECT_EQ(trunk.radius(), 0);
}

TEST(ForestTest, TreeConstructor)
{
  evs::geometric::Point center(1, 2);
  double radius = 3;
  evs::forest::Tree tree(center, radius);
  evs::geometric::Circle trunk = tree.trunk();
  EXPECT_EQ(trunk.center().x(), 1);
  EXPECT_EQ(trunk.center().y(), 2);
  EXPECT_EQ(trunk.radius(), 3);
}

TEST(ForestTest, TreeConstructorCircle)
{
  evs::geometric::Circle trunk(evs::geometric::Point(1, 2), 3);
  evs::forest::Tree tree(trunk);
  evs::geometric::Circle trunk_out = tree.trunk();
  EXPECT_EQ(trunk_out.center().x(), 1);
  EXPECT_EQ(trunk_out.center().y(), 2);
  EXPECT_EQ(trunk_out.radius(), 3);
}

TEST(ForestTest, TreeCenter)
{
  evs::geometric::Point center(1, 2);
  double radius = 3;
  evs::forest::Tree tree(center, radius);
  evs::geometric::Point center_out = tree.center();
  EXPECT_EQ(center_out.x(), 1);
  EXPECT_EQ(center_out.y(), 2);
}

TEST(ForestTest, ForestConstructor)
{
  std::tuple<double, double> x_limits(0, 10);
  std::tuple<double, double> y_limits(0, 10);
  int num_trees = 10;
  double min_radius = 1;
  double max_radius = 2;
  std::vector<evs::geometric::Circle> exclusion_zones;
  double min_spare_distance = 0;
  int max_spawn_attempts = 50;
  evs::forest::Forest forest(x_limits, y_limits, num_trees, min_radius,
                             max_radius, exclusion_zones, min_spare_distance,
                             max_spawn_attempts);
  EXPECT_LE(forest.trees().size(), num_trees);
  std::vector<evs::forest::Tree> trees = forest.trees();
  for (const evs::forest::Tree& tree : trees)
  {
    EXPECT_GE(tree.trunk().center().x(), std::get<0>(x_limits));
    EXPECT_LE(tree.trunk().center().x(), std::get<1>(x_limits));
    EXPECT_GE(tree.trunk().center().y(), std::get<0>(y_limits));
    EXPECT_LE(tree.trunk().center().y(), std::get<1>(y_limits));
    EXPECT_GE(tree.trunk().radius(), min_radius);
    EXPECT_LE(tree.trunk().radius(), max_radius);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}