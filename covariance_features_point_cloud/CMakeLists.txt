cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(calc_ft)

find_package(PCL 1.2 REQUIRED)
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
set (CMAKE_BUILD_TYPE Release)
include_directories(${PCL_INCLUDE_DIRS})
#link_directories(LINK_WITH pcl_filters pcl_io pcl_kdtree pcl_features)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (calc_ft calc_ft.cpp)
target_link_libraries (calc_ft ${PCL_LIBRARIES})
