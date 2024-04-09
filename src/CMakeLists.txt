file(GLOB HEADER_LIST CONFIGURE_DEPENDS "${PROJECT_SOURCE_DIR}/include/drone_forest/*.h")

find_package(OpenCV REQUIRED)

add_library(geometric_lib geometric.cc ${HEADER_LIST} ${OpenCV_INCLUDE_DIRS})
target_include_directories(geometric_lib PUBLIC ../include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(geometric_lib ${OpenCV_LIBS})
target_compile_features(geometric_lib PUBLIC cxx_std_17)

add_library(forest_lib forest.cc ${HEADER_LIST})
target_include_directories(forest_lib PUBLIC ../include)
target_link_libraries(forest_lib geometric_lib)
target_compile_features(forest_lib PUBLIC cxx_std_17)

add_library(lidar_lib lidar.cc ${HEADER_LIST})
target_include_directories(lidar_lib PUBLIC ../include)
target_link_libraries(lidar_lib geometric_lib)
target_compile_features(lidar_lib PUBLIC cxx_std_17)

source_group(
    TREE "${PROJECT_SOURCE_DIR}/include" 
    PREFIX "Header Files" 
    FILES ${HEADER_LIST}
)