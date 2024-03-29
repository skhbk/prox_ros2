cmake_minimum_required(VERSION 3.8)
project(prox_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "msg/MeshStamped.msg"
  "msg/TwistControllerState.msg"
  DEPENDENCIES
  std_msgs
  shape_msgs
  geometry_msgs
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
