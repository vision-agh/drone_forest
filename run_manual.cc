#include <drone_forest/drone_forest.h>

#include <iostream>

void mouseCallback(int event, int x, int y, int flags, void* userdata)
{
  if (event == cv::EVENT_LBUTTONDOWN)
  {
    std::cout << "(" << x << ", " << y << ")" << std::endl;
  }
}

int main(int argc, char** argv)
{
  // Set simulation parameters
  double sim_step = 0.1;
  std::tuple<double, double> xlim = {-10, 10};
  std::tuple<double, double> ylim = {-2, 23};
  int n_trees = 100;
  double tree_min_radius = 0.05;
  double tree_max_radius = 0.75;
  int n_lidar_beams = 36;
  double lidar_range = 3.0;
  double min_tree_spare_distance = 0.5;
  int max_spawn_attempts = 100;
  double max_speed = 1.0;
  double max_acceleration = 0.6;
  int img_height = 800;
  std::string window_name = "Drone Forest";

  // Create a simulation object
  evs::drone_forest::DroneForest sim(
      sim_step, xlim, ylim, n_trees, tree_min_radius, tree_max_radius,
      n_lidar_beams, lidar_range, min_tree_spare_distance, max_spawn_attempts,
      max_speed, max_acceleration, img_height, window_name);

  // Run the simulation in an infinite loop
  bool end_sim = false;
  while (!end_sim)
  {
    // Render the simulation
    sim.Render();

    // Check if the window is open - if not, open it
    if (cv::getWindowProperty(window_name, cv::WND_PROP_AUTOSIZE) == -1)
    {
      cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
    }
    cv::setMouseCallback(window_name, mouseCallback, NULL);

    // Show the image
    cv::imshow(window_name, sim.GetImage());
    char action = char(cv::waitKey(0));

    // Perform the action
    switch (action)
    {
      case 'w':
        sim.Step(evs::geometric::Point(0, 1));
        break;
      case 'a':
        sim.Step(evs::geometric::Point(-1, 0));
        break;
      case 's':
        sim.Step(evs::geometric::Point(0, -1));
        break;
      case 'd':
        sim.Step(evs::geometric::Point(1, 0));
        break;
      case 'q':
        end_sim = true;
        break;
      case 'r':
        sim.Reset();
        break;
      default:
        std::cout << "Unrecognized control sequence!" << std::endl;
        break;
    }
  }

  return 0;
}