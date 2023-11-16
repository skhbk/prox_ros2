cmake_minimum_required(VERSION 3.8)
project(prox2f_pregrasp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

generate_parameter_library(
  grasp_pose_publisher_params src/grasp_pose_publisher_params.yaml
)
generate_parameter_library(servo_params src/servo_params.yaml)

ament_auto_add_library(
  ${PROJECT_NAME}
  SHARED
  src/grasp_pose_publisher_node.cpp
  src/servo_node.cpp
)
target_link_libraries(${PROJECT_NAME} grasp_pose_publisher_params servo_params)

rclcpp_components_register_nodes(
  ${PROJECT_NAME}
  PLUGIN
  "prox::pregrasp::GraspPosePublisher"
  "prox::pregrasp::Servo"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE)
