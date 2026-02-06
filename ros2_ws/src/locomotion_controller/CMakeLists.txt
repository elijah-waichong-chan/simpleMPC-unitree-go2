cmake_minimum_required(VERSION 3.8)
project(locomotion_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(go2_msgs REQUIRED)
find_package(unitree_go REQUIRED)

add_library(locomotion_mpc_core
  src/configuration_state.cpp
  src/pinocchio_model.cpp
  src/gait.cpp
  src/swing_trajectory.cpp
  src/leg_controller.cpp
)

target_include_directories(locomotion_mpc_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(locomotion_mpc_core
  PUBLIC
    Eigen3::Eigen
    pinocchio::pinocchio
    pinocchio::pinocchio_parsers
)

add_executable(locomotion_node src/locomotion_node.cpp)

set_target_properties(locomotion_mpc_core locomotion_node PROPERTIES
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED YES
)

target_link_libraries(locomotion_node locomotion_mpc_core)

target_link_libraries(locomotion_node rclcpp::rclcpp)

# Ensure Pinocchio symbols are resolved when linking executables.
target_link_libraries(locomotion_node pinocchio::pinocchio pinocchio::pinocchio_parsers)

ament_target_dependencies(locomotion_node
  rclcpp
  go2_msgs
  unitree_go
)

install(TARGETS
  locomotion_mpc_core
  locomotion_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 pinocchio)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
