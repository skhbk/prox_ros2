cmake_minimum_required(VERSION 3.8)
project(prox_mesh)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(PCL REQUIRED COMPONENTS common surface)
find_package(Eigen3 REQUIRED)

generate_parameter_library(resample_mesh_params src/resample_mesh_params.yaml)

ament_auto_add_library(
  ${PROJECT_NAME}
  SHARED
  src/triangulation_node.cpp
  src/mesh_to_marker_node.cpp
  src/resample_mesh_node.cpp
)
target_link_libraries(
  ${PROJECT_NAME}
  ${PCL_LIBRARIES}
  embree3
  resample_mesh_params
)

rclcpp_components_register_nodes(
  ${PROJECT_NAME}
  PLUGIN
  "prox::mesh::Triangulation"
  "prox::mesh::MeshToMarker"
  "prox::mesh::ResampleMesh"
)

find_package(ament_cmake REQUIRED)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE)
