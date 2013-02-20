cmake_minimum_required(VERSION 2.8.3)
project(egraphs)

find_package(catkin REQUIRED roscpp)

find_package(PkgConfig)
pkg_check_modules(PC_SBPL sbpl)
include_directories(${PC_SBPL_INCLUDE_DIRS})
link_directories(${PC_SBPL_LIBRARY_DIRS})

catkin_package(INCLUDE_DIRS include src
               LIBRARIES egraphs
               DEPENDS sbpl)

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

add_library(egraphs
	    src/egraph.cpp 
	    src/egraph_2d_grid_heuristic.cpp
	    src/egraph_3d_grid_heuristic.cpp 
	    src/anytime_egraph_planner.cpp)
#rosbuild_add_executable(test_egraphs src/test_egraphs.cpp)
#target_link_libraries(test_egraphs egraphs)

target_link_libraries(egraphs ${catkin_LIBRARIES})

install(TARGETS egraphs
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/egraphs DESTINATION include FILES_MATCHING PATTERN "*.h")

