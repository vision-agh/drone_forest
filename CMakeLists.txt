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
target_link_libraries(run_manual PRIVATE drone_forest_sim)

add_executable(run_gegelati run_gegelati.cc)
target_link_libraries(run_gegelati PRIVATE gegelati_wrapper instructions json_parser ${GEGELATI_LIBRARIES})
target_compile_definitions(run_gegelati PRIVATE ROOT_DIR="${CMAKE_SOURCE_DIR}")

add_executable(eval_gegelati eval_gegelati.cc)
target_link_libraries(eval_gegelati PRIVATE gegelati_wrapper instructions json_parser geometric ${GEGELATI_LIBRARIES})
target_compile_definitions(eval_gegelati PRIVATE ROOT_DIR="${CMAKE_SOURCE_DIR}")

add_executable(codegen_gegelati codegen_gegelati.cc)
target_link_libraries(codegen_gegelati PRIVATE instructions json_parser ${GEGELATI_LIBRARIES})
target_compile_definitions(codegen_gegelati PRIVATE ROOT_DIR="${CMAKE_SOURCE_DIR}")

add_executable(render_eval render_eval.cc)
target_include_directories(render_eval PUBLIC ../include ${HEADER_LIST} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(render_eval PRIVATE forest drone json_parser ${OpenCV_LIBS})
target_compile_definitions(render_eval PRIVATE ROOT_DIR="${CMAKE_SOURCE_DIR}")