cmake_minimum_required(VERSION 3.16)
project(
  droneForest
  VERSION 0.1
  DESCRIPTION "Drone forest environment"
  LANGUAGES CXX
)

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

add_subdirectory(src)

add_subdirectory(tests)