#include <drone_forest/lidar.h>

namespace evs
{
namespace lidar
{

Lidar::Lidar(const geometric::Point& position, double range, int n_beams)
{
  position_ = position;
  range_ = range;
  beam_angles_.resize(n_beams);
  beams_.resize(n_beams);

  // Calculate the beam angles
  double angle_step = 2 * M_PI / n_beams;
  for (int i = 0; i < n_beams; i++)
  {
    beam_angles_[i] = i * angle_step;
  }

  // Initialize the beams with the maximum range
  for (int i = 0; i < n_beams; i++)
  {
    beams_[i] = geometric::Line(
        position_, position_
                       + geometric::Point(range_ * cos(beam_angles_[i]),
                                          range_ * sin(beam_angles_[i])));
  }
}

void Lidar::Draw(cv::Mat& image, geometric::Point t_vec, double m2px) const
{
  // Draw the lidar beams
  for (const auto& beam : beams_)
  {
    // NOTE: Beam has a blue color
    beam.Draw(image, cv::Scalar(255, 0, 0), t_vec, m2px);
  }
}

void Lidar::Reset(const geometric::Point& position)
{
  position_ = position;

  // Initialize the beams with the maximum range
  for (int i = 0; i < beams_.size(); i++)
  {
    beams_[i] = geometric::Line(
        position_, position_
                       + geometric::Point(range_ * cos(beam_angles_[i]),
                                          range_ * sin(beam_angles_[i])));
  }
}

std::vector<double> Lidar::Scan(const std::vector<geometric::Circle>& obstacles)
{
  // Reset the beams
  beams_.clear();
  beams_.resize(beam_angles_.size());

  // Output vector of distances
  std::vector<double> distances(beam_angles_.size(), range_);

  // Check each beam for intersections with obstacles
  for (int i = 0; i < beams_.size(); i++)
  {
    // Default: beam goes to the maximum range
    beams_[i] = geometric::Line(
        position_, position_
                       + geometric::Point(range_ * cos(beam_angles_[i]),
                                          range_ * sin(beam_angles_[i])));
    distances[i] = range_;

    for (const auto& obstacle : obstacles)
    {
      // Calculate the intersection points of the beam with the obstacle
      std::vector<geometric::Point> intersection_points =
          beams_[i].CalculateCircleIntersection(obstacle);

      // There is an intersection
      if (intersection_points.size() > 0)
      {
        // Find the closest intersection point
        double min_distance = std::numeric_limits<double>::max();
        for (const auto& intersection_point : intersection_points)
        {
          double distance = position_.Distance(intersection_point);
          if (distance < min_distance)
          {
            min_distance = distance;
          }
        }

        // Update the beam to the closest intersection point
        beams_[i] = geometric::Line(
            position_,
            position_
                + geometric::Point(min_distance * cos(beam_angles_[i]),
                                   min_distance * sin(beam_angles_[i])));
        distances[i] = min_distance;
      }
    }
  }

  return distances;
}

}  // namespace lidar
}  // namespace evs