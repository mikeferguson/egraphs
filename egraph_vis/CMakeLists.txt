cmake_minimum_required(VERSION 2.8.3)
project(egraph_vis)

find_package(catkin REQUIRED roscpp)

find_package(PkgConfig)
pkg_check_modules(PC_SBPL egraphs)
include_directories(${PC_EGRAPHS_INCLUDE_DIRS})
link_directories(${PC_EGRAPHS_LIBRARY_DIRS})

catkin_package(INCLUDE_DIRS include src
               LIBRARIES egraph_vis
               DEPENDS egraphs)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(egraph_vis
	    src/egraph_visualizer.cpp)

target_link_libraries(egraph_vis ${catkin_LIBRARIES})

install(TARGETS egraph_vis
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/egraph_vis DESTINATION include FILES_MATCHING PATTERN "*.h")

