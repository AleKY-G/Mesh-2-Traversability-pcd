cmake_minimum_required(VERSION 2.8.3)
project(mesh_to_traversability_pcd)
set(CMAKE_BUILD_TYPE RELEASE)

add_definitions(-std=c++11)

find_package(catkin_simple REQUIRED)
catkin_package(INCLUDE_DIRS include)
catkin_simple(ALL_DEPS_REQUIRED)

include_directories(${Eigen_INCLUDE_DIRS})

cs_add_library(mesh_to_traversability_pcd
  src/library/mesh_to_traversability_pcd.cpp
)

# create node to load mesh and create traversability from it
cs_add_executable(mesh_to_traversability_pcd_node
  src/mesh_to_traversability_pcd_node.cpp
  include/mesh_to_traversability_pcd/mesh_to_traversability_pcd.hpp
)
target_link_libraries(mesh_to_traversability_pcd_node mesh_to_traversability_pcd ${catkin_LIBRARIES})


#install(TARGETS mesh_to_grid_map_save_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
cs_install()
cs_export()
