#include <drone_forest/gegelati_wrapper.h>
#include <drone_forest/instructions.h>
#include <drone_forest/json_parser.h>
#include <gegelati.h>
#include <inttypes.h>
#include <math.h>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <opencv4/opencv2/opencv.hpp>
#include <string>
#include <thread>

namespace fs = std::filesystem;
using json = nlohmann::json;

void controllerLoop(std::atomic<bool>& exit, std::atomic<bool>& toggle_display,
                    std::atomic<bool>& do_eval,
                    const TPG::TPGVertex** best_root,
                    const Instructions::Set& set,
                    evs::drone_forest::GegelatiWrapper& le,
                    const Learn::LearningParameters& params)
{
  // Display preparation
  const double FPS = 30.0;
  cv::namedWindow("Drone Forest", cv::WINDOW_NORMAL);
  exit = false;

  // Execution engine setup
  Environment env(set, le.getDataSources(), params.nbRegisters,
                  params.nbProgramConstant);
  TPG::TPGExecutionEngine tee(env);

  // Main loop
  char k = 0;
  cv::Mat display = le.Render();
  int action_cnt = 0;
  while (!exit)
  {
    if (!toggle_display)
    {
      do_eval = false;
      action_cnt = 0;
    }

    if (do_eval)
    {
      // Reset environment at the beginning of the evaluation
      if (action_cnt == 0)
      {
        le.reset(0, Learn::LearningMode::VALIDATION);
      }

      // Perform an action
      auto vertexList = tee.executeFromRoot(**best_root);
      const auto actionID =
          ((const TPG::TPGAction*)vertexList.back())->getActionID();
      le.doAction(actionID);

      // Display actualization
      display = le.Render();

      // Check if the evaluation is finished
      action_cnt++;
      if (le.isTerminal() || action_cnt >= params.maxNbActionsPerEval)
      {
        do_eval = false;
        action_cnt = 0;
      }
    }

    // Show display
    cv::imshow("Drone Forest", display);
    k = char(cv::waitKey(1000.0 / FPS));
    if (k == 'q')  // Quit program
    {
      do_eval = false;
      exit = true;
    }
    else if (k == 's')  // Show evaluation
    {
      toggle_display = true;
    }
    else if (k == 'h')  // Hide evaluation
    {
      toggle_display = false;
    }
  }

  cv::destroyAllWindows();
  std::cout << "Program will end after current generation." << std::endl;
  std::cout.flush();
}

int main(int argc, char** argv)
{
  const double FPS = 30.0;

  try  // Global exception catching
  {
    std::string log_dir = "logs_tpg";
    std::string exp_str = std::to_string(std::time(nullptr));
    fs::path exp_dir = fs::path(ROOT_DIR) / log_dir / exp_str;
    fs::create_directories(exp_dir);

    std::cout << "Drone forest TPG training." << std::endl;
    std::cout << "Experiment directory: " << exp_dir << std::endl;

    // Create the instruction for programs
    Instructions::Set instruction_set;
    fillInstructionSet(instruction_set);

    // Set the parameters for the learning process from a JSON file
    fs::path params_path = fs::path(ROOT_DIR) / "params.json";
    Learn::LearningParameters params;
    File::ParametersParser::loadParametersFromJson(params_path.c_str(), params);
    std::cout << "Number of threads: " << params.nbThreads << std::endl;

    // Setup the Learning Environment (LE)
    fs::path env_config_path = fs::path(ROOT_DIR) / "env_config.json";
    std::ifstream env_config_file(env_config_path);
    json env_config = evs::drone_forest::ParseJsonFile(env_config_path);
    if (env_config["nb_directions"] != 4)
    {
      std::cerr << "Invalid number of directions in JSON file: "
                << env_config["nb_directions"] << std::endl;
      return 1;
    }
    if (int(env_config["nb_actions"]) % int(env_config["nb_directions"]) != 0)
    {
      std::cerr << "Number of actions (" << env_config["nb_actions"]
                << ") is not a multiple of the number of "
                   "directions in JSON file: "
                << env_config["nb_directions"] << std::endl;
      return 1;
    }
    int actions_per_direction =
        int(env_config["nb_actions"]) / int(env_config["nb_directions"]);
    std::vector<evs::geometric::Point> actions;
    std::vector<evs::geometric::Point> base_actions = {
        evs::geometric::Point(0.0, 1.0), evs::geometric::Point(-1.0, 0.0),
        evs::geometric::Point(0.0, -1.0), evs::geometric::Point(1.0, 0.0)};
    for (int idx_dir = 0; idx_dir < int(env_config["nb_directions"]); idx_dir++)
    {
      for (int idx_act = 0; idx_act < actions_per_direction; idx_act++)
      {
        evs::geometric::Point action = base_actions[idx_dir];
        action = action / double(actions_per_direction);
        action = action * (idx_act + 1);
        actions.push_back(action);
      }
    }
    env_config["actions"] = actions;
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
    int img_height = 800;
    std::string window_name = "Drone Forest";
    evs::drone_forest::GegelatiWrapper drone_forest_le(
        actions, sim_step, xlim, ylim, n_trees, tree_min_radius,
        tree_max_radius, n_lidar_beams, lidar_range, min_tree_spare_distance,
        max_spawn_attempts, max_speed, max_acceleration, img_height,
        window_name);
    fs::path env_out_path = exp_dir / "env_config.json";
    std::ofstream env_out(env_out_path);
    env_out << env_config << std::endl;

    // Instantiate and initialize the Learning Agent (LA)
    Learn::ParallelLearningAgent la(drone_forest_le, instruction_set, params);
    la.init();

    // Exporter for all graphs
    fs::path dot_out_path = exp_dir / "out_0000.dot";
    File::TPGGraphDotExporter dot_exporter(dot_out_path.c_str(),
                                           *la.getTPGGraph());

    // Best policy stats logger
    fs::path stats_path = exp_dir / "best_policy_stats.md";
    std::ofstream stats;
    stats.open(stats_path);
    Log::LAPolicyStatsLogger bestPolicyLogger(la, stats);

    // Export parameters before training start
    fs::path params_out_path = exp_dir / "exported_params.json";
    File::ParametersParser::writeParametersToJson(params_out_path.c_str(),
                                                  params);

    // Display thread
    std::atomic<bool> exit_program(
        true);  // Display thread will set it to false
    std::atomic<bool> toggle_display(true);
    std::atomic<bool> do_eval(false);
    const TPG::TPGVertex* best_root = nullptr;
    std::thread display_thread(controllerLoop, std::ref(exit_program),
                               std::ref(toggle_display), std::ref(do_eval),
                               &best_root, std::ref(instruction_set),
                               std::ref(drone_forest_le), std::ref(params));
    while (exit_program)
      ;  // Wait for the display thread to start

    // Train for params.nbGenerations generations
    std::cout << "Training for " << params.nbGenerations << " generations."
              << std::endl;
    std::cout << "Press 'q' with the active display window to exit."
              << std::endl;
    // Basic logger for the training process
    Log::LABasicLogger basic_logger(la);
    for (int i = 0; i < params.nbGenerations && !exit_program; i++)
    {
      char buff[13];
      sprintf(buff, "out_%04d.dot", i);
      dot_out_path = exp_dir / buff;
      dot_exporter.setNewFilePath(dot_out_path.c_str());
      dot_exporter.print();

      la.trainOneGeneration(i);

      // Evaluation of the best program
      if (!exit_program)
      {
        best_root = la.getBestRoot().first;
        do_eval = true;
        while (do_eval && !exit_program)
          ;
      }
    }

    // Keep best policy
    la.keepBestPolicy();

    // Clear introns instructions
    la.getTPGGraph()->clearProgramIntrons();

    // Export graph
    dot_out_path = exp_dir / "out_best.dot";
    dot_exporter.setNewFilePath(dot_out_path.c_str());
    dot_exporter.print();

    // Log best policy stats
    TPG::PolicyStats ps;
    ps.setEnvironment(la.getTPGGraph()->getEnvironment());
    ps.analyzePolicy(la.getBestRoot().first);
    std::ofstream best_stats;
    stats_path = exp_dir / "out_best_stats.md";
    best_stats.open(stats_path);
    best_stats << ps;
    best_stats.close();

    // Close policy stats log file
    stats.close();

    // Instruction set cleanup
    for (unsigned int i = 0; i < instruction_set.getNbInstructions(); i++)
    {
      delete (&instruction_set.getInstruction(i));
    }

    // Exit the display thread
    std::cout << "Exiting program, press a key then [enter] to exit if nothing "
                 "happens."
              << std::endl;
    display_thread.join();
  }
  catch (const std::exception& ex)
  {
    std::cerr << ex.what() << std::endl;
  }
}