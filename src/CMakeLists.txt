include(FetchContent)
FetchContent_Declare(
  json
  URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz
)
FetchContent_MakeAvailable(json)

file(GLOB HEADER_LIST CONFIGURE_DEPENDS "${PROJECT_SOURCE_DIR}/include/drone_forest/*.h")

find_package(OpenCV REQUIRED)

add_library(geometric_lib SHARED geometric.cc ${HEADER_LIST} ${OpenCV_INCLUDE_DIRS})
target_include_directories(geometric_lib PUBLIC ../include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(geometric_lib ${OpenCV_LIBS} nlohmann_json::nlohmann_json)
target_compile_features(geometric_lib PUBLIC cxx_std_17)

add_library(tree_lib SHARED tree.cc ${HEADER_LIST})
target_include_directories(tree_lib PUBLIC ../include)
target_link_libraries(tree_lib geometric_lib)
target_compile_features(tree_lib PUBLIC cxx_std_17)

add_library(forest_lib SHARED forest.cc ${HEADER_LIST})
target_include_directories(forest_lib PUBLIC ../include)
target_link_libraries(forest_lib geometric_lib tree_lib)
target_compile_features(forest_lib PUBLIC cxx_std_17)

add_library(lidar_lib SHARED lidar.cc ${HEADER_LIST})
target_include_directories(lidar_lib PUBLIC ../include)
target_link_libraries(lidar_lib geometric_lib)
target_compile_features(lidar_lib PUBLIC cxx_std_17)

add_library(drone_lib SHARED drone.cc ${HEADER_LIST})
target_include_directories(drone_lib PUBLIC ../include)
target_link_libraries(drone_lib geometric_lib lidar_lib)
target_compile_features(drone_lib PUBLIC cxx_std_17)

add_library(drone_forest_lib SHARED drone_forest.cc ${HEADER_LIST})
target_include_directories(drone_forest_lib PUBLIC ../include)
target_link_libraries(drone_forest_lib forest_lib drone_lib)
target_compile_features(drone_forest_lib PUBLIC cxx_std_17)

add_library(gegelati_wrapper_lib SHARED gegelati_wrapper.cc ${HEADER_LIST})
target_include_directories(gegelati_wrapper_lib PUBLIC ../include ${GEGELATI_INCLUDE_DIRS})
target_link_libraries(gegelati_wrapper_lib drone_forest_lib ${GEGELATI_LIBRARIES})

add_library(instructions_lib SHARED instructions.cc ${HEADER_LIST})
target_include_directories(instructions_lib PUBLIC ../include ${GEGELATI_INCLUDE_DIRS})
target_link_libraries(instructions_lib ${GEGELATI_LIBRARIES})

add_library(json_parser_lib SHARED json_parser.cc ${HEADER_LIST})
target_include_directories(json_parser_lib PUBLIC ../include)
target_link_libraries(json_parser_lib nlohmann_json::nlohmann_json)

source_group(
    TREE "${PROJECT_SOURCE_DIR}/include" 
    PREFIX "Header Files" 
    FILES ${HEADER_LIST}
)