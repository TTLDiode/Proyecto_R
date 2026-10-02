cmake_minimum_required(VERSION 3.8)
project(station3_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf launch rviz config meshes worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
