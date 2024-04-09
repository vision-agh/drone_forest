include(FetchContent)

FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)

FetchContent_MakeAvailable(googletest)
add_library(GTest::GTest INTERFACE IMPORTED)
target_link_libraries(GTest::GTest INTERFACE gtest_main)

find_package(OpenCV REQUIRED)

add_executable(geometric_test testgeometric.cc)
target_link_libraries(geometric_test PRIVATE GTest::GTest geometric_lib ${OpenCV_LIBS})
add_test(test_geometric geometric_test)

add_executable(forest_test testforest.cc)
target_link_libraries(forest_test PRIVATE GTest::GTest forest_lib ${OpenCV_LIBS})
add_test(test_forest forest_test)

add_executable(lidar_test testlidar.cc)
target_link_libraries(lidar_test PRIVATE GTest::GTest lidar_lib ${OpenCV_LIBS})
add_test(test_lidar lidar_test)