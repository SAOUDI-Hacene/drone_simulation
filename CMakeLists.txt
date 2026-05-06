cmake_minimum_required(VERSION 3.8)
project(drone_simulation)
find_package(ament_cmake REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(ros_gz_bridge REQUIRED)
install(DIRECTORY launch worlds config
  DESTINATION share/${PROJECT_NAME})
ament_package()

