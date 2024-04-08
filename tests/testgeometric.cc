#include <drone_forest/geometric.h>
#include <gtest/gtest.h>

#include <cmath>
#include <opencv4/opencv2/opencv.hpp>

TEST(GeometricTest, PointAddition)
{
  evs::geometric::Point p1(1, 2);
  evs::geometric::Point p2(3, 4);
  evs::geometric::Point p3 = p1 + p2;
  EXPECT_EQ(p3.x(), 4);
  EXPECT_EQ(p3.y(), 6);
}

TEST(GeometricTest, PointSubtraction)
{
  evs::geometric::Point p1(1, 2);
  evs::geometric::Point p2(3, 4);
  evs::geometric::Point p3 = p1 - p2;
  EXPECT_EQ(p3.x(), -2);
  EXPECT_EQ(p3.y(), -2);
}

TEST(GeometricTest, PointScaleUp)
{
  evs::geometric::Point p1(1, 2);
  double scale = 2;
  evs::geometric::Point p3 = p1 * scale;
  EXPECT_EQ(p3.x(), 2);
  EXPECT_EQ(p3.y(), 4);
}

TEST(GeometricTest, PointEquality)
{
  evs::geometric::Point p1(1, 2);
  evs::geometric::Point p2(1, 2);
  EXPECT_TRUE(p1 == p2);
}

TEST(GeometricTest, PointDistance)
{
  evs::geometric::Point p1(1, 2);
  evs::geometric::Point p2(4, 6);
  double distance = p1.Distance(p2);
  EXPECT_EQ(distance, 5);
}

// TEST(GeometricTest, CircleDraw)
// {
//   cv::Mat img(100, 100, CV_8UC3, cv::Scalar(255, 255, 255));
//   evs::geometric::Circle circle(evs::geometric::Point(50, 50), 10);
//   circle.Draw(img, cv::Scalar(0, 0, 0));
//   cv::Vec3b color = img.at<cv::Vec3b>(50, 50);
//   EXPECT_EQ(color[0], 0);
//   EXPECT_EQ(color[1], 0);
//   EXPECT_EQ(color[2], 0);
// }

TEST(GeometricTest, LineClosestPointOnLine)
{
  evs::geometric::Point p1(1, 1);
  evs::geometric::Point p2(3, 3);
  evs::geometric::Line line(p1, p2);
  evs::geometric::Point p3(2, 2);
  evs::geometric::Point closest_point = line.CalculateClosestPoint(p3);
  EXPECT_EQ(closest_point.x(), 2);
  EXPECT_EQ(closest_point.y(), 2);
}

TEST(GeometricTest, LineClosestPointOffLine)
{
  evs::geometric::Point p1(1, 1);
  evs::geometric::Point p2(3, 3);
  evs::geometric::Line line(p1, p2);
  evs::geometric::Point p3(4, 4);
  evs::geometric::Point closest_point = line.CalculateClosestPoint(p3);
  EXPECT_EQ(closest_point.x(), 3);
  EXPECT_EQ(closest_point.y(), 3);
}

TEST(GeometricTest, LineClosestPointAside)
{
  evs::geometric::Point p1(1, 1);
  evs::geometric::Point p2(3, 3);
  evs::geometric::Line line(p1, p2);
  evs::geometric::Point p3(0, 3);
  evs::geometric::Point closest_point = line.CalculateClosestPoint(p3);
  EXPECT_EQ(closest_point.x(), 1.5);
  EXPECT_EQ(closest_point.y(), 1.5);
}

TEST(GeometricTest, LineCircleIntersection)
{
  evs::geometric::Point p1(1, 1);
  evs::geometric::Point p2(3, 3);
  evs::geometric::Line line(p1, p2);

  // Line start inside circle
  evs::geometric::Circle circle(p1, 1);
  std::vector<evs::geometric::Point> intersection_points =
      line.CalculateCircleIntersection(circle);
  EXPECT_EQ(intersection_points.size(), 1);
  EXPECT_DOUBLE_EQ(intersection_points[0].x(), 1 + sqrt(2) / 2);
  EXPECT_DOUBLE_EQ(intersection_points[0].y(), 1 + sqrt(2) / 2);

  // Line end inside circle
  circle = evs::geometric::Circle(p2, 1);
  intersection_points = line.CalculateCircleIntersection(circle);
  EXPECT_EQ(intersection_points.size(), 1);
  EXPECT_DOUBLE_EQ(intersection_points[0].x(), 3 - sqrt(2) / 2);
  EXPECT_DOUBLE_EQ(intersection_points[0].y(), 3 - sqrt(2) / 2);

  // Line inside circle
  circle = evs::geometric::Circle(evs::geometric::Point(2, 2), 3);
  intersection_points = line.CalculateCircleIntersection(circle);
  EXPECT_EQ(intersection_points.size(), 0);

  // Line tangent to circle
  line = evs::geometric::Line(evs::geometric::Point(0, 3), p2);
  circle = evs::geometric::Circle(evs::geometric::Point(2, 2), 1);
  intersection_points = line.CalculateCircleIntersection(circle);
  EXPECT_EQ(intersection_points.size(), 1);
  EXPECT_DOUBLE_EQ(intersection_points[0].x(), 2);
  EXPECT_DOUBLE_EQ(intersection_points[0].y(), 3);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}