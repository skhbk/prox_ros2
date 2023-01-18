cmake_minimum_required(VERSION 3.8)
project(prox2f_contact_analysis)

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(PCL REQUIRED COMPONENTS common)
find_package(CGAL REQUIRED)

ament_auto_add_executable(sim_state_publisher src/sim_state_publisher_node.cpp)
target_link_libraries(sim_state_publisher ${PCL_LIBRARIES})

ament_auto_add_library(
  ${PROJECT_NAME}
  SHARED
  src/resample_cloud_node.cpp
  src/contact_mapping_node.cpp
  src/robotiq_2f_85_fingertip.cpp
  src/virtual_wrench_node.cpp
)
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} CGAL::CGAL)

rclcpp_components_register_nodes(
  ${PROJECT_NAME}
  PLUGIN
  "prox::contact::ResampleCloud"
  "prox::contact::ContactMapping"
  "prox::contact::VirtualWrench"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE)
