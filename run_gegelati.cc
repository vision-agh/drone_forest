#include <drone_forest/gegelati_wrapper.h>
#include <gegelati.h>

#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

int main(int argc, char **argv)
{
  try
  {  // Global exception catching.

    std::cout << "Drone forest TPG training." << std::endl;

    // Create the instruction instructionSet for programs
    Instructions::Set instructionSet;
    auto minus = [](double a, double b) -> double { return a - b; };
    auto add = [](double a, double b) -> double { return a + b; };
    auto mult = [](double a, double b) -> double { return a * b; };
    auto div = [](double a, double b) -> double { return a / b; };
    auto max = [](double a, double b) -> double { return std::max(a, b); };
    auto ln = [](double a) -> double { return std::log(a); };
    auto exp = [](double a) -> double { return std::exp(a); };

    instructionSet.add(*(new Instructions::LambdaInstruction<double, double>(
        minus, "$0 = $1 - $2;")));
    instructionSet.add(*(new Instructions::LambdaInstruction<double, double>(
        add, "$0 = $1 + $2;")));
    instructionSet.add(*(new Instructions::LambdaInstruction<double, double>(
        mult, "$0 = $1 * $2;")));
    instructionSet.add(*(new Instructions::LambdaInstruction<double, double>(
        div, "$0 = $1 / $2;")));
    instructionSet.add(*(new Instructions::LambdaInstruction<double, double>(
        max, "$0 = (($1) < ($2)) ? ($2) : ($1);")));
    instructionSet.add(
        *(new Instructions::LambdaInstruction<double>(exp, "$0 = exp($1);")));
    instructionSet.add(
        *(new Instructions::LambdaInstruction<double>(ln, "$0 = log($1);")));

    // Set the parameters for the learning process.
    // Controls mutations probability, program lengths, and graph size among
    // other things. Loads them from the file params.json
    Learn::LearningParameters params;
    File::ParametersParser::loadParametersFromJson(ROOT_DIR "/params.json",
                                                   params);

    // Setup the Learning Environment (LE)
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
    evs::drone_forest::GegelatiWrapper droneForestLE(
        sim_step, xlim, ylim, n_trees, tree_min_radius, tree_max_radius,
        n_lidar_beams, lidar_range, min_tree_spare_distance, max_spawn_attempts,
        max_speed, max_acceleration, img_height, window_name);

    // Instantiate and initialize the Learning Agent (LA)
    Learn::LearningAgent la(droneForestLE, instructionSet, params);
    la.init();

    // // Start display thread
    // std::atomic<bool> exitProgram =
    //     true;  // (set to false by other thread after init)
    // std::atomic<bool> doDisplay = false;
    // std::atomic<uint64_t> generation = 0;
    // std::deque<std::tuple<uint64_t, double, double>> replay;
    // std::thread threadDisplay(Renderer::replayThread, std::ref(exitProgram),
    //                           std::ref(doDisplay), std::ref(generation),
    //                           pendulumLE.p.TIME_DELTA, std::ref(replay));
    // while (exitProgram)
    //   ;  // Wait for other thread to print key info.

    // Basic logger for the training process
    Log::LABasicLogger basicLogger(la);

    // Train for params.nbGenerations generations
    for (int i = 0; i < params.nbGenerations; i++)
    {
      la.trainOneGeneration(i);

      //   // Get replay of best root actions on the pendulum
      //   replay = createReplay(pendulumLE, la.getBestRoot().first,
      //   instructionSet,
      //                         params);
      //   generation = i;

      //   // trigger display
      //   doDisplay = true;
      //   while (doDisplay && !exitProgram)
      //     ;
    }

    // Cleanup instruction set
    // deleteInstructions(instructionSet);

    // Exit the display thread
    std::cout << "Exiting program, press a key then [enter] to exit if nothing "
                 "happens.";
    // threadDisplay.join();
  }
  catch (const std::exception &ex)
  {
    std::cerr << ex.what() << std::endl;
  }
}