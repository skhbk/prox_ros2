cmake_minimum_required(VERSION 3.5)
project(prox_preprocess)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(
  ${PROJECT_NAME}
  SHARED
  src/concatenate_clouds_node.cpp
  src/ema_node.cpp
  src/pass_through_node.cpp
  src/outlier_removal_node.cpp
)
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})

rclcpp_components_register_nodes(
  ${PROJECT_NAME}
  PLUGIN
  "prox::ConcatenateClouds"
  "prox::EMA"
  "prox::PassThrough"
  "prox::OutlierRemoval"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE)
