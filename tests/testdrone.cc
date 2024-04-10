#include <drone_forest/drone.h>
#include <gtest/gtest.h>

#include <cmath>

TEST(DroneTest, DroneConstructor)
{
  evs::geometric::Point position(1, 2);
  double lidar_range = 3;
  int lidar_n_beams = 4;
  evs::drone::Drone drone(position, lidar_range, lidar_n_beams);
  EXPECT_DOUBLE_EQ(drone.position().x(), 1);
  EXPECT_DOUBLE_EQ(drone.position().y(), 2);
  EXPECT_DOUBLE_EQ(drone.lidar().position().x(), 1);
  EXPECT_DOUBLE_EQ(drone.lidar().position().y(), 2);
  EXPECT_DOUBLE_EQ(drone.lidar().range(), 3);
  EXPECT_DOUBLE_EQ(drone.lidar().angles().size(), 4);
  EXPECT_DOUBLE_EQ(drone.lidar().beams().size(), 4);
}

TEST(DroneTest, DroneMove)
{
  evs::geometric::Point position(1, 2);
  double lidar_range = 3;
  int lidar_n_beams = 4;
  evs::drone::Drone drone(position, lidar_range, lidar_n_beams);
  evs::geometric::Point velocity(1, 1);
  drone.Move(1, velocity);
  EXPECT_DOUBLE_EQ(drone.position().x(), 2);
  EXPECT_DOUBLE_EQ(drone.position().y(), 3);
  EXPECT_DOUBLE_EQ(drone.lidar().position().x(), 2);
  EXPECT_DOUBLE_EQ(drone.lidar().position().y(), 3);
}

TEST(DroneTest, DroneReset)
{
  evs::geometric::Point position(1, 2);
  double lidar_range = 3;
  int lidar_n_beams = 4;
  evs::drone::Drone drone(position, lidar_range, lidar_n_beams);
  evs::geometric::Point velocity(1, 1);
  drone.Move(1, velocity);
  EXPECT_NE(drone.position().x(), 1);
  EXPECT_NE(drone.position().y(), 2);
  drone.Reset(position);
  EXPECT_DOUBLE_EQ(drone.position().x(), 1);
  EXPECT_DOUBLE_EQ(drone.position().y(), 2);
  EXPECT_DOUBLE_EQ(drone.lidar().position().x(), 1);
  EXPECT_DOUBLE_EQ(drone.lidar().position().y(), 2);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}