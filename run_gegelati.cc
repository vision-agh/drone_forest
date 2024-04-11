#include <drone_forest/gegelati_wrapper.h>
#include <gegelati.h>
#include <inttypes.h>
#include <math.h>

#include <chrono>
#include <iomanip>
#include <iostream>
#include <opencv4/opencv2/opencv.hpp>
#include <string>
#include <thread>
#define _USE_MATH_DEFINES  // To get M_PI (is it necessary?)

void displayEnv(std::atomic<bool> &exit, cv::Mat &display)
{
  const double FPS = 30.0;
  cv::namedWindow("Drone Forest", cv::WINDOW_NORMAL);
  std::cout << "Press 'q' on the open display to exit." << std::endl;
  std::cout.flush();
  exit = false;

  char k = 0;
  while (!exit)
  {
    cv::imshow("Drone Forest", display);
    k = char(cv::waitKey(1));
    if (k == 'q')
    {
      exit = true;
    }
  }

  cv::destroyAllWindows();
  std::cout << "Program will end after current generation." << std::endl;
  std::cout.flush();
}

int main(int argc, char **argv)
{
  const double FPS = 30.0;

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
    auto cos = [](double a) -> double { return std::cos(a); };
    auto sin = [](double a) -> double { return std::sin(a); };
    auto tan = [](double a) -> double { return std::tan(a); };
    auto pi = [](double a) -> double { return M_PI; };
    auto multByConst = [](double a, Data::Constant c) -> double
    { return a * (double)c / 10.0; };

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
    instructionSet.add(
        *(new Instructions::LambdaInstruction<double>(cos, "$0 = cos($1);")));
    instructionSet.add(
        *(new Instructions::LambdaInstruction<double>(sin, "$0 = sin($1);")));
    instructionSet.add(
        *(new Instructions::LambdaInstruction<double>(tan, "$0 = tan($1);")));
    instructionSet.add(
        *(new Instructions::LambdaInstruction<double>(pi, "$0 = M_PI;")));
    instructionSet.add(
        *(new Instructions::LambdaInstruction<double, Data::Constant>(
            multByConst, "$0 = $1 * ((double)($2) / 10.0);")));

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

    // Basic logger for the training process
    Log::LABasicLogger basicLogger(la);

    // Display thread
    std::atomic<bool> exit(true);  // Display thread will set it to false
    cv::Mat display = droneForestLE.Render();
    std::thread displayThread(displayEnv, std::ref(exit), std::ref(display));
    while (exit)
      ;  // Wait for the display thread to start

    // Train for params.nbGenerations generations
    Environment env(instructionSet, droneForestLE.getDataSources(),
                    params.nbRegisters, params.nbProgramConstant);
    TPG::TPGExecutionEngine tee(env);
    for (int i = 0; i < params.nbGenerations && !exit; i++)
    {
      la.trainOneGeneration(i);

      // Evaluation of the best program
      droneForestLE.reset(0, Learn::LearningMode::VALIDATION);
      while (!droneForestLE.isTerminal() && !exit)
      {
        auto vertexList = tee.executeFromRoot(*la.getBestRoot().first);
        const auto actionID =
            ((const TPG::TPGAction *)vertexList.back())->getActionID();
        droneForestLE.doAction(actionID);
        display = droneForestLE.Render();
        std::this_thread::sleep_for(
            std::chrono::milliseconds(static_cast<int>(1000.0 / FPS)));
      }
    }

    // Cleanup instruction set
    // deleteInstructions(instructionSet);

    // Exit the display thread
    displayThread.join();
    std::cout << "Exiting program, press a key then [enter] to exit if nothing "
                 "happens."
              << std::endl;
  }
  catch (const std::exception &ex)
  {
    std::cerr << ex.what() << std::endl;
  }
}