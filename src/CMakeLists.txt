include(FetchContent)
FetchContent_Declare(
  json
  URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz
)
FetchContent_MakeAvailable(json)

file(GLOB HEADER_LIST CONFIGURE_DEPENDS "${PROJECT_SOURCE_DIR}/include/drone_forest/*.h")

find_package(OpenCV REQUIRED)

add_library(geometric SHARED geometric/point.cc geometric/shape.cc geometric/circle.cc geometric/line.cc geometric/rectangle.cc ${HEADER_LIST} ${OpenCV_INCLUDE_DIRS})
target_include_directories(geometric PUBLIC ../include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(geometric ${OpenCV_LIBS} nlohmann_json::nlohmann_json)
target_compile_features(geometric PUBLIC cxx_std_17)

add_library(forest SHARED forest.cc tree.cc ${HEADER_LIST})
target_include_directories(forest PUBLIC ../include)
target_link_libraries(forest geometric)
target_compile_features(forest PUBLIC cxx_std_17)

add_library(lidar SHARED lidar.cc ${HEADER_LIST})
target_include_directories(lidar PUBLIC ../include)
target_link_libraries(lidar geometric)
target_compile_features(lidar PUBLIC cxx_std_17)

add_library(drone SHARED drone.cc ${HEADER_LIST})
target_include_directories(drone PUBLIC ../include)
target_link_libraries(drone geometric lidar)
target_compile_features(drone PUBLIC cxx_std_17)

add_library(drone_forest_lib SHARED drone_forest.cc ${HEADER_LIST})
target_include_directories(drone_forest_lib PUBLIC ../include)
target_link_libraries(drone_forest_lib forest drone)
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