#include <drone_forest/drone_forest.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <opencv4/opencv2/opencv.hpp>
#include <tuple>

namespace py = pybind11;

PYBIND11_MODULE(drone_forest, m)
{
  py::class_<evs::drone_forest::DroneForest>(m, "DroneForest")
      .def(py::init<double, std::tuple<double, double>,
                    std::tuple<double, double>, double, int, double, double,
                    int, double, double, int, double, double, double, double,
                    int, std::string>(),
           py::arg("sim_step"), py::arg("x_lim"), py::arg("y_lim"),
           py::arg("goal_y"), py::arg("n_trees"), py::arg("tree_min_radius"),
           py::arg("tree_max_radius"), py::arg("n_lidar_beams"),
           py::arg("lidar_range"), py::arg("min_tree_spare_distance"),
           py::arg("max_spawn_attempts"), py::arg("max_speed"),
           py::arg("max_acceleration"), py::arg("drone_width_m"),
           py::arg("drone_height_m"), py::arg("img_height") = 800,
           py::arg("window_name") = "Drone Forest")
      .def("check_collision", &evs::drone_forest::DroneForest::CheckCollision,
           "Check if the drone is colliding with a tree")
      .def("get_drone_position",
           &evs::drone_forest::DroneForest::GetDronePositionAsVector,
           "Get the drone position")
      .def("get_lidar_distances",
           &evs::drone_forest::DroneForest::GetLidarDistancesAsVector,
           "Get the LiDAR distances")
      .def("get_image", &evs::drone_forest::DroneForest::GetImageAsVector,
           "Get the rendered image as a vector of bytes")
      .def("get_image_size", &evs::drone_forest::DroneForest::GetImageSize,
           "Get the size of the rendered image")
      .def("get_time", &evs::drone_forest::DroneForest::GetTime,
           "Get the current simulation time")
      .def("render", &evs::drone_forest::DroneForest::Render,
           "Render the environment")
      .def("reset",
           static_cast<void (evs::drone_forest::DroneForest::*)()>(
               &evs::drone_forest::DroneForest::Reset),
           "Reset the environment")
      .def("reset_seed",
           static_cast<void (evs::drone_forest::DroneForest::*)(int)>(
               &evs::drone_forest::DroneForest::Reset),
           "Reset the environment with a given seed", py::arg("seed"))
      .def("step", &evs::drone_forest::DroneForest::StepVelocityVector,
           "Step the environment with a given action", py::arg("velocity_vec"));
}