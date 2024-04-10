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

add_library(drone_lib drone.cc ${HEADER_LIST})
target_include_directories(drone_lib PUBLIC ../include)
target_link_libraries(drone_lib geometric_lib lidar_lib)
target_compile_features(drone_lib PUBLIC cxx_std_17)

add_library(drone_forest_lib drone_forest.cc ${HEADER_LIST})
target_include_directories(drone_forest_lib PUBLIC ../include)
target_link_libraries(drone_forest_lib forest_lib drone_lib)
target_compile_features(drone_forest_lib PUBLIC cxx_std_17)

add_library(gegelati_wrapper_lib gegelati_wrapper.cc ${HEADER_LIST})
target_include_directories(gegelati_wrapper_lib PUBLIC ../include ${GEGELATI_INCLUDE_DIRS})
target_link_libraries(gegelati_wrapper_lib drone_forest_lib ${GEGELATI_LIBRARIES})

source_group(
    TREE "${PROJECT_SOURCE_DIR}/include" 
    PREFIX "Header Files" 
    FILES ${HEADER_LIST}
)