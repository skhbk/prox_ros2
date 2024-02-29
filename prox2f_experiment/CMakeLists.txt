cmake_minimum_required(VERSION 3.8)
project(prox2f_experiment)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

generate_parameter_library(
  experiment_base_params src/experiment_base_params.yaml
)

ament_auto_add_library(${PROJECT_NAME} SHARED src/experiment_base.cpp)
target_link_libraries(${PROJECT_NAME} experiment_base_params)

ament_auto_add_executable(pickup src/pickup.cpp)
ament_auto_add_executable(pregrasp_evaluation src/pregrasp_evaluation.cpp)
ament_auto_add_executable(pregrasp_demo src/pregrasp_demo.cpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE)
