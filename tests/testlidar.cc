#include <drone_forest/lidar.h>
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

TEST(LidarTest, LidarConstructor)
{
  evs::geometric::Point position(1, 2);
  double range = 3;
  int n_beams = 4;
  std::vector<double> ref_angles = {0, M_PI / 2, M_PI, 3 * M_PI / 2};
  evs::lidar::Lidar lidar(position, range, n_beams);
  EXPECT_DOUBLE_EQ(lidar.position().x(), 1);
  EXPECT_DOUBLE_EQ(lidar.position().y(), 2);
  EXPECT_DOUBLE_EQ(lidar.range(), 3);
  EXPECT_DOUBLE_EQ(lidar.angles().size(), 4);
  EXPECT_DOUBLE_EQ(lidar.beams().size(), 4);
  std::vector<double> angles = lidar.angles();
  for (int i = 0; i < n_beams; i++)
  {
    EXPECT_DOUBLE_EQ(angles[i], ref_angles[i]);
  }
  std::vector<evs::geometric::Line> beams = lidar.beams();
  for (int i = 0; i < n_beams; i++)
  {
    EXPECT_DOUBLE_EQ(beams[i].start().x(), 1);
    EXPECT_DOUBLE_EQ(beams[i].start().y(), 2);
    EXPECT_DOUBLE_EQ(beams[i].end().x(), 1 + 3 * cos(ref_angles[i]));
    EXPECT_DOUBLE_EQ(beams[i].end().y(), 2 + 3 * sin(ref_angles[i]));
  }
}

TEST(LidarTest, LidarUpdatePosition)
{
  evs::geometric::Point position(1, 2);
  double range = 3;
  int n_beams = 4;
  evs::lidar::Lidar lidar(position, range, n_beams);
  EXPECT_DOUBLE_EQ(lidar.position().x(), 1);
  EXPECT_DOUBLE_EQ(lidar.position().y(), 2);
  evs::geometric::Point new_position(3, 4);
  evs::geometric::Point old_position = lidar.UpdatePosition(new_position);
  EXPECT_DOUBLE_EQ(old_position.x(), 1);
  EXPECT_DOUBLE_EQ(old_position.y(), 2);
  EXPECT_DOUBLE_EQ(lidar.position().x(), 3);
  EXPECT_DOUBLE_EQ(lidar.position().y(), 4);
}

TEST(LidarTest, LidarScan)
{
  // No obstacles
  evs::geometric::Point position(1, 2);
  double range = 3;
  int n_beams = 4;
  evs::lidar::Lidar lidar(position, range, n_beams);
  std::vector<evs::geometric::Circle> obstacles;
  std::vector<double> distances = lidar.Scan(obstacles);
  for (int i = 0; i < n_beams; i++)
  {
    EXPECT_DOUBLE_EQ(distances[i], range);
  }

  // Obstacle at the end of the beam
  obstacles.push_back(evs::geometric::Circle(
      evs::geometric::Point(1 + 3 * cos(M_PI / 2), 2 + 3 * sin(M_PI / 2)),
      0.5));
  distances = lidar.Scan(obstacles);
  for (int i = 0; i < n_beams; i++)
  {
    if (i == 1)
    {
      EXPECT_DOUBLE_EQ(distances[i], 3 - 0.5);
    }
    else
    {
      EXPECT_DOUBLE_EQ(distances[i], range);
    }
  }

  // Beam through the obstacle
  obstacles.clear();
  obstacles.push_back(
      evs::geometric::Circle(evs::geometric::Point(0, 4), sqrt(2)));
  distances = lidar.Scan(obstacles);
  for (int i = 0; i < n_beams; i++)
  {
    if (i == 1)
    {
      EXPECT_DOUBLE_EQ(distances[i], 1.0);
    }
    else
    {
      EXPECT_DOUBLE_EQ(distances[i], range);
    }
  }
}

TEST(LidarTest, LidarDraw)
{
  // No origin translation, no scaling
  evs::geometric::Point position(1, 2);
  double range = 3;
  int n_beams = 4;
  evs::lidar::Lidar lidar(position, range, n_beams);
  cv::Mat image(100, 100, CV_8UC3, cv::Scalar(255, 255, 255));
  evs::geometric::Point t_vec(0, 0);
  double m2px = 1;
  lidar.Draw(image, t_vec, m2px);
  cv::Vec3b center = image.at<cv::Vec3b>(99 - 2, 1);
  EXPECT_EQ(center[0], 255);
  EXPECT_EQ(center[1], 0);
  EXPECT_EQ(center[2], 0);
  cv::Vec3b right = image.at<cv::Vec3b>(99 - 2, 4);
  EXPECT_EQ(right[0], 255);
  EXPECT_EQ(right[1], 0);
  EXPECT_EQ(right[2], 0);
  cv::Vec3b left = image.at<cv::Vec3b>(99 - 2, 0);
  EXPECT_EQ(left[0], 255);
  EXPECT_EQ(left[1], 0);
  EXPECT_EQ(left[2], 0);
  cv::Vec3b up = image.at<cv::Vec3b>(99 - 5, 1);
  EXPECT_EQ(up[0], 255);
  EXPECT_EQ(up[1], 0);
  EXPECT_EQ(up[2], 0);
  cv::Vec3b down = image.at<cv::Vec3b>(99, 1);
  EXPECT_EQ(down[0], 255);
  EXPECT_EQ(down[1], 0);
  EXPECT_EQ(down[2], 0);

  // Origin translation, scaling
  image = cv::Mat(100, 100, CV_8UC3, cv::Scalar(255, 255, 255));
  t_vec = evs::geometric::Point(25, 25);
  m2px = 2;
  lidar.Draw(image, t_vec, m2px);
  center = image.at<cv::Vec3b>(45, 52);
  EXPECT_EQ(center[0], 255);
  EXPECT_EQ(center[1], 0);
  EXPECT_EQ(center[2], 0);
  right = image.at<cv::Vec3b>(45, 58);
  EXPECT_EQ(right[0], 255);
  EXPECT_EQ(right[1], 0);
  EXPECT_EQ(right[2], 0);
  left = image.at<cv::Vec3b>(45, 46);
  EXPECT_EQ(left[0], 255);
  EXPECT_EQ(left[1], 0);
  EXPECT_EQ(left[2], 0);
  up = image.at<cv::Vec3b>(39, 52);
  EXPECT_EQ(up[0], 255);
  EXPECT_EQ(up[1], 0);
  EXPECT_EQ(up[2], 0);
  down = image.at<cv::Vec3b>(51, 52);
  EXPECT_EQ(down[0], 255);
  EXPECT_EQ(down[1], 0);
  EXPECT_EQ(down[2], 0);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}