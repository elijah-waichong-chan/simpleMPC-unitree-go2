cmake_minimum_required(VERSION 3.5)

project(go2_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch meshes urdf xacro dae
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
