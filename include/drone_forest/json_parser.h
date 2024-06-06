#ifndef _DRONE_FOREST_JSON_PARSER_H_
#define _DRONE_FOREST_JSON_PARSER_H_

#include <drone_forest/geometric/point.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>

namespace fs = std::filesystem;
using json = nlohmann::json;

namespace evs
{
namespace drone_forest
{

/**
 * @brief Create a vector of actions as 2D velocity vectors
 *
 * @param nb_actions Number of actions
 * @param nb_directions Number of directions
 * @return std::vector<evs::geometric::Point> Vector of actions
 */
std::vector<evs::geometric::Point> createActionVec(int nb_actions,
                                                   int nb_directions);

/**
 * @brief Parse a JSON file and return the content as a JSON object
 *
 * @param file_path Path to the JSON file
 * @return json JSON object
 */
json ParseJsonFile(const fs::path& file_path);

}  // namespace drone_forest
}  // namespace evs

#endif  // _DRONE_FOREST_JSON_PARSER_H_