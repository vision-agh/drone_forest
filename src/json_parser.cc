#include <drone_forest/json_parser.h>

namespace evs
{
namespace drone_forest
{

std::vector<evs::geometric::Point> createActionVec(int nb_actions,
                                                   int nb_directions)
{
  std::vector<evs::geometric::Point> actions;
  std::vector<evs::geometric::Point> base_actions = {
      evs::geometric::Point(0.0, 1.0), evs::geometric::Point(-1.0, 0.0),
      evs::geometric::Point(0.0, -1.0), evs::geometric::Point(1.0, 0.0)};

  // Generate actions as 2D velocity vectors
  int actions_per_direction = nb_actions / nb_directions;
  for (int idx_dir = 0; idx_dir < nb_directions; idx_dir++)
  {
    for (int idx_act = 0; idx_act < actions_per_direction; idx_act++)
    {
      evs::geometric::Point action = base_actions[idx_dir];
      action = action / double(actions_per_direction);
      action = action * (idx_act + 1);
      actions.push_back(action);
    }
  }

  return actions;
}

json ParseJsonFile(const fs::path& file_path)
{
  std::ifstream config_file(file_path);
  json j = json::parse(config_file);

  if (!j.contains("nb_directions"))
  {
    std::cerr << "Number of directions not found in JSON file." << std::endl;
    j["nb_directions"] = 4;
    std::cerr << "Setting number of directions to default: "
              << j["nb_directions"] << std::endl;
  }

  if (!j.contains("nb_actions"))
  {
    std::cerr << "Number of actions not found in JSON file." << std::endl;
    j["nb_actions"] = 4;
    std::cerr << "Setting number of actions to default: " << j["nb_actions"]
              << std::endl;
  }

  if (!j.contains("actions"))
  {
    std::cerr << "Actions not found in JSON file." << std::endl;
    j["actions"] = createActionVec(j["nb_actions"], j["nb_directions"]);
    std::cerr << "Setting actions to default: " << j["actions"] << std::endl;
  }

  if (!j.contains("sim_step"))
  {
    std::cerr << "Simulation step not found in JSON file." << std::endl;
    j["sim_step"] = 0.1;
    std::cerr << "Setting simulation step to default: " << j["sim_step"]
              << std::endl;
  }

  if (!j.contains("x_lim"))
  {
    std::cerr << "X-axis limits not found in JSON file." << std::endl;
    j["x_lim"]["min"] = -10;
    j["x_lim"]["max"] = 10;
    std::cerr << "Setting x-axis limits to default: " << j["x_lim"]
              << std::endl;
  }
  else
  {
    if (!j["x_lim"].contains("min"))
    {
      std::cerr << "X-axis minimum limit not found in JSON file." << std::endl;
      j["x_lim"]["min"] = -10;
      std::cerr << "Setting x-axis minimum limit to default: "
                << j["x_lim"]["min"] << std::endl;
    }

    if (!j["x_lim"].contains("max"))
    {
      std::cerr << "X-axis maximum limit not found in JSON file." << std::endl;
      j["x_lim"]["max"] = 10;
      std::cerr << "Setting x-axis maximum limit to default: "
                << j["x_lim"]["max"] << std::endl;
    }
  }

  if (!j.contains("y_lim"))
  {
    std::cerr << "Y-axis limits not found in JSON file." << std::endl;
    j["y_lim"]["min"] = -2;
    j["y_lim"]["max"] = 23;
    std::cerr << "Setting y-axis limits to default: " << j["y_lim"]
              << std::endl;
  }
  else
  {
    if (!j["y_lim"].contains("min"))
    {
      std::cerr << "Y-axis minimum limit not found in JSON file." << std::endl;
      j["y_lim"]["min"] = -2;
      std::cerr << "Setting y-axis minimum limit to default: "
                << j["y_lim"]["min"] << std::endl;
    }

    if (!j["y_lim"].contains("max"))
    {
      std::cerr << "Y-axis maximum limit not found in JSON file." << std::endl;
      j["y_lim"]["max"] = 23;
      std::cerr << "Setting y-axis maximum limit to default: "
                << j["y_lim"]["max"] << std::endl;
    }
  }

  if (!j.contains("y_static_limit"))
  {
    std::cerr << "Y-axis static limit not found in JSON file." << std::endl;
    j["y_static_limit"] = j["y_lim"]["max"];
    std::cerr << "Setting y-axis static limit to default: "
              << j["y_static_limit"] << std::endl;
  }

  if (!j.contains("n_trees"))
  {
    std::cerr << "Number of trees not found in JSON file." << std::endl;
    j["n_trees"] = 100;
    std::cerr << "Setting number of trees to default: " << j["n_trees"]
              << std::endl;
  }

  if (!j.contains("tree_radius_lim"))
  {
    std::cerr << "Tree radius limits not found in JSON file." << std::endl;
    j["tree_radius_lim"]["min"] = 0.05;
    j["tree_radius_lim"]["max"] = 0.75;
    std::cerr << "Setting tree radius limits to default: "
              << j["tree_radius_lim"] << std::endl;
  }
  else
  {
    if (!j["tree_radius_lim"].contains("min"))
    {
      std::cerr << "Tree minimum radius not found in JSON file." << std::endl;
      j["tree_radius_lim"]["min"] = 0.05;
      std::cerr << "Setting tree minimum radius to default: "
                << j["tree_radius_lim"]["min"] << std::endl;
    }

    if (!j["tree_radius_lim"].contains("max"))
    {
      std::cerr << "Tree maximum radius not found in JSON file." << std::endl;
      j["tree_radius_lim"]["max"] = 0.75;
      std::cerr << "Setting tree maximum radius to default: "
                << j["tree_radius_lim"]["max"] << std::endl;
    }
  }

  if (!j.contains("n_lidar_beams"))
  {
    std::cerr << "Number of LiDAR beams not found in JSON file." << std::endl;
    j["n_lidar_beams"] = 36;
    std::cerr << "Setting number of LiDAR beams to default: "
              << j["n_lidar_beams"] << std::endl;
  }

  if (!j.contains("lidar_range"))
  {
    std::cerr << "LiDAR range not found in JSON file." << std::endl;
    j["lidar_range"] = 3.0;
    std::cerr << "Setting LiDAR range to default: " << j["lidar_range"]
              << std::endl;
  }

  if (!j.contains("min_tree_spare_distance"))
  {
    std::cerr << "Minimum tree spare distance not found in JSON file."
              << std::endl;
    j["min_tree_spare_distance"] = 0.5;
    std::cerr << "Setting minimum tree spare distance to default: "
              << j["min_tree_spare_distance"] << std::endl;
  }

  if (!j.contains("max_spawn_attempts"))
  {
    std::cerr << "Maximum spawn attempts not found in JSON file." << std::endl;
    j["max_spawn_attempts"] = 100;
    std::cerr << "Setting maximum spawn attempts to default: "
              << j["max_spawn_attempts"] << std::endl;
  }

  if (!j.contains("max_speed"))
  {
    std::cerr << "Maximum speed not found in JSON file." << std::endl;
    j["max_speed"] = 1.0;
    std::cerr << "Setting maximum speed to default: " << j["max_speed"]
              << std::endl;
  }

  if (!j.contains("max_acceleration"))
  {
    std::cerr << "Maximum acceleration not found in JSON file." << std::endl;
    j["max_acceleration"] = 0.6;
    std::cerr << "Setting maximum acceleration to default: "
              << j["max_acceleration"] << std::endl;
  }

  if (!j.contains("drone_width"))
  {
    std::cerr << "Drone width not found in JSON file." << std::endl;
    j["drone_width"] = 0.1;
    std::cerr << "Setting drone width to default: " << j["drone_width"]
              << std::endl;
  }

  if (!j.contains("drone_height"))
  {
    std::cerr << "Drone height not found in JSON file." << std::endl;
    j["drone_height"] = 0.2;
    std::cerr << "Setting drone height to default: " << j["drone_height"]
              << std::endl;
  }

  return j;
}

}  // namespace drone_forest
}  // namespace evs