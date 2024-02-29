cmake_minimum_required(VERSION 3.8)
project(prox2f_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(OpenCV REQUIRED)

generate_parameter_library(
  camera_transform_publisher_params src/camera_transform_publisher_params.yaml
)

ament_auto_add_library(
  ${PROJECT_NAME} SHARED src/camera_transform_publisher_node.cpp
)
target_link_libraries(
  ${PROJECT_NAME} ${OpenCV_LIBRARIES} camera_transform_publisher_params
)

rclcpp_components_register_nodes(
  ${PROJECT_NAME} PLUGIN "prox::camera::CameraTransformPublisher"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE)
