#include <drone_forest/instructions.h>
#include <drone_forest/json_parser.h>
#include <gegelati.h>

#include <filesystem>
#include <iostream>

namespace fs = std::filesystem;
using json = nlohmann::json;

int main(int argc, char** argv)
{
  // Expect the path to the experiment directory
  std::string exp_dir;
  if (argc < 2)
  {
    std::cerr << "No experiment directory provided." << std::endl;
    std::cerr << "Running for the newest experiment." << std::endl;
    fs::path log_dir = fs::path(ROOT_DIR) / "logs_tpg";
    for (const auto& entry : fs::directory_iterator(log_dir))
    {
      if (fs::is_directory(entry))
      {
        if (entry.path() > exp_dir)
        {
          exp_dir = entry.path();
        }
      }
    }
  }
  else
  {
    exp_dir = argv[1];
  }

  // Load the experiment directory
  fs::path exp_path =
      exp_dir[0] == '/' ? fs::path(exp_dir) : fs::path(ROOT_DIR) / exp_dir;

  std::cout << "Generate C code from pre-trained dot file." << std::endl;
  std::cout << "Experiment directory: " << exp_path << std::endl;

  // Create a set of instructions
  Instructions::Set instruction_set;
  fillInstructionSet(instruction_set);

  // Get parameters
  fs::path params_path = exp_path / "exported_params.json";
  Learn::LearningParameters params;
  File::ParametersParser::loadParametersFromJson(params_path.c_str(), params);

  // Get environment configuration
  fs::path env_config_path = exp_path / "env_config.json";
  std::ifstream env_config_file(env_config_path);
  json env_config = evs::drone_forest::ParseJsonFile(env_config_path);

  // Create the environment
  Data::PrimitiveTypeArray<double> input{env_config["n_lidar_beams"]};
  std::vector<std::reference_wrapper<const Data::DataHandler>> input_data{
      input};

  Environment dot_env(instruction_set, input_data, params.nbRegisters,
                      params.nbProgramConstant);
  TPG::TPGGraph dot_graph(dot_env);

  // Load the best program from the experiment directory
  fs::path best_program_path = exp_path / "out_best.dot";
  File::TPGGraphDotImporter dot(best_program_path.c_str(), dot_env, dot_graph);
  dot.importGraph();

  // Generate C code
  fs::path c_code_path = exp_path / "codegen" / "";
  fs::create_directories(c_code_path);
  CodeGen::TPGGenerationEngineFactory factory(
      CodeGen::TPGGenerationEngineFactory::switchMode);
  std::unique_ptr<CodeGen::TPGGenerationEngine> tpg_gen =
      factory.create("out_best", dot_graph, c_code_path.c_str());
  tpg_gen->generateTPGGraph();
}