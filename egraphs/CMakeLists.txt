cmake_minimum_required(VERSION 2.8.3)
project(egraphs)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

find_package(PkgConfig)
pkg_check_modules(PC_SBPL sbpl)
include_directories(${PC_SBPL_INCLUDE_DIRS})
link_directories(${PC_SBPL_LIBRARY_DIRS})

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
)

catkin_package(
  INCLUDE_DIRS
    include
    src
  LIBRARIES
    egraphs
  DEPENDS
    sbpl
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library(egraphs
  src/egraph.cpp
  src/egraph_2d_grid_heuristic.cpp
  src/egraph_3d_grid_heuristic.cpp
  src/egraph_euclidean_heuristic.cpp
)
target_link_libraries(egraphs
  ${catkin_LIBRARIES}
  ${SBPL_LIBRARIES}
)

install(
  TARGETS egraphs
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
  DIRECTORY include/egraphs
  DESTINATION include FILES_MATCHING PATTERN "*.h"
)
