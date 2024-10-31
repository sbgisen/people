cmake_minimum_required(VERSION 3.8)
project(people_tracking_filter)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
link_directories(${BFL_LIBRARY_DIRS})
include_directories(${BFL_INCLUDE_DIRS})

find_package(Boost REQUIRED COMPONENTS thread)

ament_auto_add_library(
  ${PROJECT_NAME}
  src/uniform_vector.cpp
  src/gaussian_vector.cpp
  src/gaussian_pos_vel.cpp
  src/mcpdf_pos_vel.cpp
  src/mcpdf_vector.cpp
  src/sysmodel_pos_vel.cpp
  src/sysmodel_vector.cpp
  src/measmodel_pos.cpp
  src/measmodel_vector.cpp
  src/tracker_particle.cpp
  src/tracker_kalman.cpp
  src/detector_particle.cpp
)

ament_auto_add_executable(people_tracker src/people_tracking_node.cpp)

target_link_libraries(
  people_tracker
  ${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${BFL_LIBRARIES}
)

ament_auto_package(INSTALL_TO_SHARE launch)
