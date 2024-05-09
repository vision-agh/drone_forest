#include <drone_forest/forest.h>
#include <gtest/gtest.h>

#include <cmath>
// #include <opencv4/opencv2/opencv.hpp>

TEST(ForestTest, TreeDefaultConstructor)
{
  evs::forest::Tree tree;
  evs::geometric::Circle trunk = tree.Trunk();
  EXPECT_EQ(trunk.Center().x(), 0);
  EXPECT_EQ(trunk.Center().y(), 0);
  EXPECT_EQ(trunk.Radius(), 0);
}

TEST(ForestTest, TreeConstructor)
{
  evs::geometric::Point center(1, 2);
  double radius = 3;
  evs::forest::Tree tree(center, radius);
  evs::geometric::Circle trunk = tree.Trunk();
  EXPECT_EQ(trunk.Center().x(), 1);
  EXPECT_EQ(trunk.Center().y(), 2);
  EXPECT_EQ(trunk.Radius(), 3);
}

TEST(ForestTest, TreeConstructorCircle)
{
  evs::geometric::Circle trunk(evs::geometric::Point(1, 2), 3);
  evs::forest::Tree tree(trunk);
  evs::geometric::Circle trunk_out = tree.Trunk();
  EXPECT_EQ(trunk_out.Center().x(), 1);
  EXPECT_EQ(trunk_out.Center().y(), 2);
  EXPECT_EQ(trunk_out.Radius(), 3);
}

TEST(ForestTest, TreeCenter)
{
  evs::geometric::Point center(1, 2);
  double radius = 3;
  evs::forest::Tree tree(center, radius);
  evs::geometric::Point center_out = tree.Trunk().Center();
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
  EXPECT_LE(forest.NumberOfTrees(), num_trees);
  std::vector<evs::forest::Tree> trees = forest.Trees();
  for (const evs::forest::Tree& tree : trees)
  {
    EXPECT_GE(tree.Trunk().Center().x(), std::get<0>(x_limits));
    EXPECT_LE(tree.Trunk().Center().x(), std::get<1>(x_limits));
    EXPECT_GE(tree.Trunk().Center().y(), std::get<0>(y_limits));
    EXPECT_LE(tree.Trunk().Center().y(), std::get<1>(y_limits));
    EXPECT_GE(tree.Trunk().Radius(), min_radius);
    EXPECT_LE(tree.Trunk().Radius(), max_radius);
  }
}

TEST(ForestTest, ForestExclusionZones)
{
  std::tuple<double, double> x_limits(0, 10);
  std::tuple<double, double> y_limits(0, 10);
  int num_trees = 10;
  double min_radius = 1;
  double max_radius = 2;
  std::vector<evs::geometric::Circle> exclusion_zones;
  exclusion_zones.push_back(
      evs::geometric::Circle(evs::geometric::Point(5, 5), 1));
  exclusion_zones.push_back(
      evs::geometric::Circle(evs::geometric::Point(1, 1), 2));
  exclusion_zones.push_back(
      evs::geometric::Circle(evs::geometric::Point(8, 8), 3));
  double min_spare_distance = 0;
  int max_spawn_attempts = 50;
  evs::forest::Forest forest(x_limits, y_limits, num_trees, min_radius,
                             max_radius, exclusion_zones, min_spare_distance,
                             max_spawn_attempts);
  std::vector<evs::forest::Tree> trees = forest.Trees();
  for (const evs::forest::Tree& tree : trees)
  {
    bool in_exclusion_zone = false;
    for (const evs::geometric::Circle& exclusion_zone : exclusion_zones)
    {
      if (tree.Trunk().Center().Distance(exclusion_zone.Center())
          < tree.Trunk().Radius() + exclusion_zone.Radius())
      {
        in_exclusion_zone = true;
        break;
      }
    }
    EXPECT_FALSE(in_exclusion_zone);
  }
}

TEST(ForestTest, ForestTreesDistances)
{
  std::tuple<double, double> x_limits(0, 10);
  std::tuple<double, double> y_limits(0, 10);
  int num_trees = 50;
  double min_radius = 1;
  double max_radius = 2;
  std::vector<evs::geometric::Circle> exclusion_zones;
  double min_spare_distance = 0;
  int max_spawn_attempts = 50;
  evs::forest::Forest forest(x_limits, y_limits, num_trees, min_radius,
                             max_radius, exclusion_zones, min_spare_distance,
                             max_spawn_attempts);
  std::vector<evs::forest::Tree> trees = forest.Trees();
  for (int i = 0; i < trees.size(); i++)
  {
    for (int j = i + 1; j < trees.size(); j++)
    {
      double distance =
          trees[i].Trunk().Center().Distance(trees[j].Trunk().Center());
      EXPECT_GE(distance, trees[i].Trunk().Radius() + trees[j].Trunk().Radius()
                              + min_spare_distance);
    }
  }
}

TEST(ForestTest, ForestSize)
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
  EXPECT_EQ(forest.NumberOfTrees(), forest.Trees().size());
}

TEST(ForestTest, ForestDraw)
{
  std::tuple<double, double> x_limits(0, 200);
  std::tuple<double, double> y_limits(0, 200);
  int num_trees = 10;
  double min_radius = 0.5;
  double max_radius = 10;
  std::vector<evs::geometric::Circle> exclusion_zones;
  double min_spare_distance = 0;
  int max_spawn_attempts = 50;
  evs::forest::Forest forest(x_limits, y_limits, num_trees, min_radius,
                             max_radius, exclusion_zones, min_spare_distance,
                             max_spawn_attempts);
  cv::Mat img(200, 200, CV_8UC3, cv::Scalar(0, 255, 0));
  evs::geometric::Point t_vec(0, 0);
  double m2px = 1;
  forest.Draw(img, t_vec, m2px);
}

TEST(ForestTest, ForestSetSeed)
{
  std::tuple<double, double> x_limits(0, 10);
  std::tuple<double, double> y_limits(0, 10);
  int num_trees = 10;
  double min_radius = 1;
  double max_radius = 2;
  std::vector<evs::geometric::Circle> exclusion_zones;
  double min_spare_distance = 0;
  int max_spawn_attempts = 50;
  evs::forest::Forest::SetSeed(0);
  evs::forest::Forest forest1(x_limits, y_limits, num_trees, min_radius,
                              max_radius, exclusion_zones, min_spare_distance,
                              max_spawn_attempts);
  evs::forest::Forest::SetSeed(0);
  evs::forest::Forest forest2(x_limits, y_limits, num_trees, min_radius,
                              max_radius, exclusion_zones, min_spare_distance,
                              max_spawn_attempts);
  EXPECT_EQ(forest1.Trees().size(), forest2.Trees().size());
  for (int i = 0; i < forest1.Trees().size(); i++)
  {
    EXPECT_EQ(forest1.Trees()[i].Trunk().Center().x(),
              forest2.Trees()[i].Trunk().Center().x());
    EXPECT_EQ(forest1.Trees()[i].Trunk().Center().y(),
              forest2.Trees()[i].Trunk().Center().y());
    EXPECT_EQ(forest1.Trees()[i].Trunk().Radius(),
              forest2.Trees()[i].Trunk().Radius());
  }
}

TEST(ForestTest, ForestGetObstacles)
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
  std::vector<evs::geometric::Circle> obstacles = forest.GetObstacles();
  for (const evs::geometric::Circle& obstacle : obstacles)
  {
    bool in_forest = false;
    for (const evs::forest::Tree& tree : forest.Trees())
    {
      if (obstacle.Center() == tree.Trunk().Center()
          && obstacle.Radius() == tree.Trunk().Radius())
      {
        in_forest = true;
        break;
      }
    }
    EXPECT_TRUE(in_forest);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}