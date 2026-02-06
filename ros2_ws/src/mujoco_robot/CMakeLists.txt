cmake_minimum_required(VERSION 3.8)
project(mujoco_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(unitree_go REQUIRED)
find_package(go2_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Viewer deps
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)

# ---- Find MuJoCo (system-wide install) ----
find_path(MUJOCO_INCLUDE_DIR
  NAMES mujoco/mujoco.h
  PATHS /usr/include /usr/local/include
)

find_library(MUJOCO_LIBRARY
  NAMES mujoco libmujoco
  PATHS /usr/lib /usr/local/lib /usr/lib/x86_64-linux-gnu
)

if(NOT MUJOCO_INCLUDE_DIR OR NOT MUJOCO_LIBRARY)
  message(FATAL_ERROR
    "Could not find system MuJoCo.\n"
    "Set MUJOCO_INCLUDE_DIR and MUJOCO_LIBRARY, e.g.\n"
    "  colcon build --cmake-args -DMUJOCO_INCLUDE_DIR=/path/include -DMUJOCO_LIBRARY=/path/libmujoco.so"
  )
endif()

# ---- Build the node ----
add_executable(mujoco_robot_node src/mujoco_robot_node.cpp)

target_include_directories(mujoco_robot_node PRIVATE ${MUJOCO_INCLUDE_DIR})

target_link_libraries(mujoco_robot_node
  ${MUJOCO_LIBRARY}
  Eigen3::Eigen
  OpenGL::GL
  glfw
)

ament_target_dependencies(mujoco_robot_node
  rclcpp
  std_msgs
  unitree_go
  go2_msgs
  Eigen3
)

install(TARGETS mujoco_robot_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
