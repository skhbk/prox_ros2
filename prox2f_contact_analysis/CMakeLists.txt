cmake_minimum_required(VERSION 3.5)
project(prox2f_contact_analysis)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
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
  contact_mapping
  SHARED
  src/contact_mapping_node.cpp
  src/robotiq_2f_85_fingertip.cpp
)

target_link_libraries(contact_mapping ${PCL_LIBRARIES} CGAL::CGAL)

rclcpp_components_register_nodes(
  contact_mapping PLUGIN "prox::contact::ContactMapping"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE)
