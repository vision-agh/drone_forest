cmake_minimum_required(VERSION 3.16)
project(
  droneForest
  VERSION 0.1
  DESCRIPTION "Drone forest environment"
  LANGUAGES CXX
)

find_package(GEGELATI)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

include(CTest)

# find_package(Doxygen)
# if(Doxygen_FOUND)
#   add_subdirectory(docs)
# else()
#   message(STATUS "Doxygen not found, not building documentation")
# endif()

add_subdirectory(extern/pybind11)
add_subdirectory(scripts)
add_subdirectory(src)
add_subdirectory(tests)

add_executable(run_manual run_manual.cc)
target_link_libraries(run_manual PRIVATE drone_forest_lib)

add_executable(run_gegelati run_gegelati.cc)
target_link_libraries(run_gegelati PRIVATE gegelati_wrapper_lib instructions_lib json_parser_lib ${GEGELATI_LIBRARIES})
target_compile_definitions(run_gegelati PRIVATE ROOT_DIR="${CMAKE_SOURCE_DIR}")

add_executable(eval_gegelati eval_gegelati.cc)
target_link_libraries(eval_gegelati PRIVATE gegelati_wrapper_lib instructions_lib json_parser_lib ${GEGELATI_LIBRARIES})
target_compile_definitions(eval_gegelati PRIVATE ROOT_DIR="${CMAKE_SOURCE_DIR}")