#ifndef _DRONE_FOREST_JSON_PARSER_H_
#define _DRONE_FOREST_JSON_PARSER_H_

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
 * @brief Parse a JSON file and return the content as a JSON object
 *
 * @param file_path Path to the JSON file
 * @return json JSON object
 */
json ParseJsonFile(const fs::path& file_path);

}  // namespace drone_forest
}  // namespace evs

#endif  // _DRONE_FOREST_JSON_PARSER_H_