cmake_minimum_required(VERSION 3.8)
project(station2_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf launch rviz config meshes worlds scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
