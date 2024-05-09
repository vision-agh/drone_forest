#include <drone_forest/drone.h>
#include <gtest/gtest.h>

#include <cmath>

TEST(DroneTest, DroneConstructor)
{
  evs::geometric::Point position(1, 2);
  double lidar_range = 3;
  int lidar_n_beams = 4;
  evs::drone::Drone drone(position, lidar_range, lidar_n_beams);
  EXPECT_DOUBLE_EQ(drone.Position().x(), 1);
  EXPECT_DOUBLE_EQ(drone.Position().y(), 2);
  EXPECT_DOUBLE_EQ(drone.Lidar().Position().x(), 1);
  EXPECT_DOUBLE_EQ(drone.Lidar().Position().y(), 2);
  EXPECT_DOUBLE_EQ(drone.Lidar().Range(), 3);
  EXPECT_DOUBLE_EQ(drone.Lidar().Angles().size(), 4);
  EXPECT_DOUBLE_EQ(drone.Lidar().Beams().size(), 4);
}

TEST(DroneTest, DroneMove)
{
  evs::geometric::Point position(1, 2);
  double lidar_range = 3;
  int lidar_n_beams = 4;
  evs::drone::Drone drone(position, lidar_range, lidar_n_beams, 1.0, 0.6);
  evs::geometric::Point velocity(1, 1);
  drone.Move(1, velocity);
  EXPECT_DOUBLE_EQ(drone.Position().x(), 1.6);
  EXPECT_DOUBLE_EQ(drone.Position().y(), 2.6);
  EXPECT_DOUBLE_EQ(drone.Lidar().Position().x(), 1.6);
  EXPECT_DOUBLE_EQ(drone.Lidar().Position().y(), 2.6);
}

TEST(DroneTest, DroneReset)
{
  evs::geometric::Point position(1, 2);
  double lidar_range = 3;
  int lidar_n_beams = 4;
  evs::drone::Drone drone(position, lidar_range, lidar_n_beams);
  evs::geometric::Point velocity(1, 1);
  drone.Move(1, velocity);
  EXPECT_NE(drone.Position().x(), 1);
  EXPECT_NE(drone.Position().y(), 2);
  drone.Reset(position);
  EXPECT_DOUBLE_EQ(drone.Position().x(), 1);
  EXPECT_DOUBLE_EQ(drone.Position().y(), 2);
  EXPECT_DOUBLE_EQ(drone.Lidar().Position().x(), 1);
  EXPECT_DOUBLE_EQ(drone.Lidar().Position().y(), 2);
}

TEST(DroneTest, DroneLidarScan)
{
  evs::geometric::Point position(1, 2);
  double lidar_range = 3;
  int lidar_n_beams = 4;
  evs::drone::Drone drone(position, lidar_range, lidar_n_beams);
  std::vector<evs::geometric::Circle> obstacles;
  obstacles.push_back(evs::geometric::Circle(evs::geometric::Point(1, 4), 1));
  obstacles.push_back(evs::geometric::Circle(evs::geometric::Point(1, 0), 1));
  obstacles.push_back(evs::geometric::Circle(evs::geometric::Point(3, 2), 1));
  obstacles.push_back(evs::geometric::Circle(evs::geometric::Point(-1, 2), 1));
  std::vector<double> scan = drone.LidarScan(obstacles);
  EXPECT_DOUBLE_EQ(scan[0], 1);
  EXPECT_DOUBLE_EQ(scan[1], 1);
  EXPECT_DOUBLE_EQ(scan[2], 1);
  EXPECT_DOUBLE_EQ(scan[3], 1);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}