#include <drone_forest/gegelati_wrapper.h>
#include <drone_forest/instructions.h>
#include <drone_forest/json_parser.h>
#include <gegelati.h>

#include <algorithm>
#include <filesystem>
#include <iomanip>
#include <iostream>

namespace fs = std::filesystem;

int main(int argc, char** argv)
{
  size_t seed = argc > 1 ? std::stoul(argv[1]) : 0;
  fs::path save_eval_dir = argc > 2 ? fs::path(argv[2]) : fs::path();
  const double FPS = 30.0;

  // Path to the experiment directory
  fs::path log_dir = fs::path(ROOT_DIR) / "logs_tpg";

  // List all experiments directories inside the log directory
  std::cout << "Experiments located in: " << log_dir << std::endl;
  std::cout << "Select experiment to evaluate: " << std::endl;
  std::vector<fs::path> exp_paths;
  for (const auto& entry : fs::directory_iterator(log_dir))
  {
    if (fs::is_directory(entry))
    {
      exp_paths.push_back(entry.path());
    }
  }
  std::sort(exp_paths.begin(), exp_paths.end());
  for (int idx = 0; idx < exp_paths.size(); idx++)
  {
    fs::path exp = exp_paths[idx];
    std::string exp_dir;
    for (const auto& dir : exp)
    {
      exp_dir = dir;
    }
    std::time_t exp_time = std::stod(exp_dir);
    std::cout << std::setw(3) << idx + 1 << "\t" << exp_dir << "\t"
              << std::put_time(std::localtime(&exp_time), "%Y-%m-%d %H:%M:%S")
              << std::endl;
  }

  // Choose the experiment to evaluate
  int exp_idx = exp_paths.size();
  while (exp_idx < 0 || exp_idx >= exp_paths.size())
  {
    std::cout << "Select experiment number: ";
    std::cin >> exp_idx;
    exp_idx--;
  }
  fs::path exp_dir = exp_paths[exp_idx];
  std::cout << "Selected experiment: " << exp_dir << std::endl;

  // Create the set of instructions
  Instructions::Set set;
  fillInstructionSet(set);

  // Create the environment
  fs::path env_config_path = exp_dir / "env_config.json";
  std::ifstream env_config_file(env_config_path);
  json env_config = evs::drone_forest::ParseJsonFile(env_config_path);
  std::vector<evs::geometric::Point> actions;
  if (env_config["nb_actions"] != 4 && env_config["nb_actions"] != 8)
  {
    std::cerr << "Invalid number of actions in JSON file: "
              << env_config["nb_actions"] << std::endl;
    return 1;
  }
  for (const auto& action : env_config["actions"])
  {
    actions.push_back(evs::geometric::Point(action["x"], action["y"]));
  }
  double sim_step = env_config["sim_step"];
  std::tuple<double, double> xlim = {env_config["x_lim"]["min"],
                                     env_config["x_lim"]["max"]};
  std::tuple<double, double> ylim = {env_config["y_lim"]["min"],
                                     env_config["y_lim"]["max"]};
  int n_trees = env_config["n_trees"];
  double tree_min_radius = env_config["tree_radius_lim"]["min"];
  double tree_max_radius = env_config["tree_radius_lim"]["max"];
  int n_lidar_beams = env_config["n_lidar_beams"];
  double lidar_range = env_config["lidar_range"];
  double min_tree_spare_distance = env_config["min_tree_spare_distance"];
  int max_spawn_attempts = env_config["max_spawn_attempts"];
  double max_speed = env_config["max_speed"];
  double max_acceleration = env_config["max_acceleration"];
  double drone_width = env_config["drone_width"];
  double drone_height = env_config["drone_height"];
  int img_height = 800;
  std::string window_name = "Drone Forest";
  evs::drone_forest::GegelatiWrapper drone_forest_le(
      actions, sim_step, xlim, ylim, n_trees, tree_min_radius, tree_max_radius,
      n_lidar_beams, lidar_range, min_tree_spare_distance, max_spawn_attempts,
      max_speed, max_acceleration, drone_width, drone_height, img_height,
      window_name);

  // Load parameters from the experiment directory
  fs::path params_path = exp_dir / "exported_params.json";
  Learn::LearningParameters params;
  File::ParametersParser::loadParametersFromJson(params_path.c_str(), params);

  // Load the best program from the experiment directory
  fs::path best_program_path = exp_dir / "out_best.dot";
  Environment dot_env(set, drone_forest_le.getDataSources(), params.nbRegisters,
                      params.nbProgramConstant);
  TPG::TPGGraph dot_graph(dot_env);
  File::TPGGraphDotImporter dot_importer(best_program_path.c_str(), dot_env,
                                         dot_graph);
  dot_importer.importGraph();

  // Setup evaluation
  drone_forest_le.reset(seed, Learn::LearningMode::VALIDATION);
  TPG::TPGExecutionEngine tee(dot_env);
  const TPG::TPGVertex* root(dot_graph.getRootVertices().back());
  cv::namedWindow(window_name, cv::WINDOW_NORMAL);
  cv::Mat display;
  char k = 0;

  // Evaluate the best program
  for (size_t i = 0;
       i < params.maxNbActionsPerEval && !drone_forest_le.isTerminal(); i++)
  {
    // Perform action
    auto vertex_list = tee.executeFromRoot(*root);
    const auto action_id =
        ((const TPG::TPGAction*)vertex_list.back())->getActionID();
    drone_forest_le.doAction(action_id);

    // Render the environment
    display = drone_forest_le.Render();
    if (!save_eval_dir.empty())
    {
      fs::path img_path = save_eval_dir / (std::to_string(i) + ".png");
      cv::imwrite(img_path.string(), display);
    }
    cv::imshow(window_name, display);
    k = char(cv::waitKey(1000 / FPS));
    if (k == 'q')
    {
      break;
    }
  }
  std::cin.clear();
  std::cout << "Evaluation ended. Press any key with the focus on the window "
            << window_name << " to close the program." << std::endl;
  cv::waitKey(0);
  cv::destroyAllWindows();

  return 0;
}