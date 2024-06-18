#include <drone_forest/forest.h>
#include <drone_forest/geometric/circle.h>
#include <drone_forest/geometric/line.h>
#include <drone_forest/geometric/point.h>
#include <drone_forest/json_parser.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <opencv4/opencv2/opencv.hpp>

namespace fs = std::filesystem;

/**
 * @brief Application to render the given drone movement history.
 *
 * This application has following input arguments:
 * --eval-dir: Path to the evaluation directory (required)
 * --input-img: Path to the input image file (optional, when not provided, a new
 * image will be created)
 * --path-color: Color of the path (optional, default: red)
 * --seed: Seed for the random number generator (optional, default: 0)
 *
 * @param argc Number of arguments
 * @param argv Arguments
 * @return int Exit code
 */
int main(int argc, char** argv)
{
  // Parse input arguments
  fs::path eval_dir;
  fs::path input_img;
  cv::Scalar path_color = cv::Scalar(0, 0, 255);  // Default: red
  int seed = 0;
  for (int i = 1; i < argc; i++)
  {
    if (std::string(argv[i]) == "--eval-dir")
    {
      if (i + 1 < argc && fs::is_directory(fs::path(argv[i + 1])))
      {
        eval_dir = fs::path(argv[i + 1]);
      }
      else
      {
        std::cerr << "Invalid evaluation directory provided." << std::endl;
        return 1;
      }
    }
    else if (std::string(argv[i]) == "--input-img")
    {
      if (i + 1 < argc && fs::is_regular_file(fs::path(argv[i + 1])))
      {
        input_img = fs::path(argv[i + 1]);
      }
      else
      {
        std::cerr << "Invalid input image file provided." << std::endl;
        return 1;
      }
    }
    else if (std::string(argv[i]) == "--path-color")
    {
      if (i + 1 < argc)
      {
        std::string color = argv[i + 1];
        if (color == "red")
        {
          path_color = cv::Scalar(0, 0, 255);
        }
        else if (color == "blue")
        {
          path_color = cv::Scalar(255, 0, 0);
        }
        else
        {
          std::cerr << "Invalid path color provided." << std::endl;
          return 1;
        }
      }
      else
      {
        std::cerr << "Invalid path color provided." << std::endl;
        return 1;
      }
    }
    else if (std::string(argv[i]) == "--seed")
    {
      if (i + 1 < argc)
      {
        seed = std::stoi(argv[i + 1]);
      }
      else
      {
        std::cerr << "Invalid seed provided." << std::endl;
        return 1;
      }
    }
  }

  // Check if the experiment directory is provided
  if (eval_dir.empty())
  {
    std::cerr << "Evaluation directory is required." << std::endl;
    return 1;
  }

  // Load the path to the experiment directory
  fs::path exp_dir_file = eval_dir / "exp_dir.txt";
  if (!fs::exists(exp_dir_file))
  {
    std::cerr << "Experiment directory file not found." << std::endl;
    return 1;
  }
  std::ifstream exp_file(exp_dir_file);
  fs::path exp_dir;
  exp_file >> exp_dir;
  exp_file.close();

  // Load the environment configuration
  fs::path env_config_path = exp_dir / "env_config.json";
  if (!fs::exists(env_config_path))
  {
    std::cerr << "Environment configuration file not found." << std::endl;
    return 1;
  }
  json env_config = evs::drone_forest::ParseJsonFile(env_config_path);

  // Render the forest image if not provided
  std::tuple<double, double> xlim = {env_config["x_lim"]["min"],
                                     env_config["x_lim"]["max"]};
  std::tuple<double, double> ylim = {env_config["y_lim"]["min"],
                                     env_config["y_lim"]["max"]};
  evs::geometric::Point t_vec = {-std::get<0>(xlim), -std::get<0>(ylim)};
  int img_height = 800;
  double m2px;
  cv::Mat img;
  std::string window_name = "Drone Forest";
  if (input_img.empty())
  {
    // Create forest
    double y_static_limit = env_config["y_static_limit"];
    int n_trees = env_config["n_trees"];
    double tree_min_radius = env_config["tree_radius_lim"]["min"];
    double tree_max_radius = env_config["tree_radius_lim"]["max"];
    double min_tree_spare_distance = env_config["min_tree_spare_distance"];
    int max_spawn_attempts = env_config["max_spawn_attempts"];
    evs::forest::Forest::SetSeed(seed);
    evs::forest::Forest forest(
        xlim, ylim, y_static_limit, n_trees, tree_min_radius, tree_max_radius,
        {evs::geometric::Circle(evs::geometric::Point(0, 0), 1.0)},
        min_tree_spare_distance);

    // Render image
    double y_range = std::get<1>(ylim) - std::get<0>(ylim);
    double x_range = std::get<1>(xlim) - std::get<0>(xlim);
    m2px = img_height / y_range;
    double img_width = int(x_range * m2px);
    img = cv::Mat(img_height, img_width, CV_8UC3, cv::Scalar(54, 149, 70));
    forest.Draw(img, t_vec, m2px);
  }
  else
  {
    // Load the input image
    img = cv::imread(input_img.string());
    if (img.empty())
    {
      std::cerr << "Failed to load the input image." << std::endl;
      return 1;
    }
    img_height = img.rows;
    m2px = img.rows / (std::get<1>(ylim) - std::get<0>(ylim));
  }

  // Load the drone movement history
  fs::path drone_history_path = eval_dir / "drone_pos.csv";
  if (!fs::exists(drone_history_path))
  {
    std::cerr << "Drone movement history file not found." << std::endl;
    return 1;
  }
  std::ifstream drone_history_file(drone_history_path);
  std::vector<evs::geometric::Point> drone_positions;
  std::string line;
  int i = 1;
  while (std::getline(drone_history_file, line))
  {
    std::istringstream iss(line);
    double x, y;
    char trash;
    iss >> x;
    iss >> trash;
    iss >> y;
    drone_positions.push_back(evs::geometric::Point(x, y));
  }
  drone_history_file.close();

  // Render the drone movement history
  for (const auto& drone_position : drone_positions)
  {
    cv::Point p = drone_position.ToCvPoint(t_vec, m2px, img_height);
    cv::circle(img, p, 5, path_color, cv::FILLED);
    cv::imshow(window_name, img);
    char k = cv::waitKey(10);
    if (k == 'q')
    {
      break;
    }
  }

  // Add start point and finish line
  cv::Scalar misc_color = cv::Scalar(0, 0, 0);  // Gold
  evs::geometric::Point start_point(0, 0);
  cv::Point p = start_point.ToCvPoint(t_vec, m2px, img_height);
  cv::circle(img, p, 5, misc_color, cv::FILLED);
  evs::geometric::Point goal_left_end(std::get<0>(xlim),
                                      std::get<1>(ylim) - 2.0);
  evs::geometric::Point goal_right_end(std::get<1>(xlim),
                                       std::get<1>(ylim) - 2.0);
  evs::geometric::Line goal_line(goal_left_end, goal_right_end);
  goal_line.Draw(img, misc_color, t_vec, m2px);

  cv::imshow(window_name, img);
  cv::waitKey(0);
  fs::path output_path = eval_dir / "output.png";
  cv::imwrite(output_path.string(), img);
  cv::destroyAllWindows();

  return 0;
}