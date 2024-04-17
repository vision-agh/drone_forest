pybind11_add_module(drone_forest src/drone_forest_py.cc)
target_include_directories(drone_forest PUBLIC ../include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(drone_forest PRIVATE drone_forest_lib ${OpenCV_LIBS})