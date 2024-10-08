cmake_minimum_required(VERSION 3.8)
project(people_tracking_filter)

# find_package(ament_cmake_auto REQUIRED)
# ament_auto_find_build_dependencies()
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(people_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Look for bfl (Bayesian Filtering Library)
find_package(PkgConfig)
pkg_check_modules(
  BFL
  REQUIRED
  orocos-bfl
)
link_directories(${BFL_LIBRARY_DIRS})

find_package(Boost REQUIRED COMPONENTS thread)

# list(
#   APPEND
#   ${PROJECT_NAME}_FOUND_INCLUDE_DIRS
#   ${BFL_INCLUDE_DIRS}/bfl
# )

# list(
#   APPEND
#   ${PROJECT_NAME}_FOUND_LIBRARIES
#   ${Boost_LIBRARIES}
#   ${BFL_LIBRARIES}
# )
# ament_auto_add_library(
#   ${PROJECT_NAME}
#   src/uniform_vector.cpp
#   src/gaussian_vector.cpp
#   src/gaussian_pos_vel.cpp
#   src/mcpdf_pos_vel.cpp
#   src/mcpdf_vector.cpp
#   src/sysmodel_pos_vel.cpp
#   src/sysmodel_vector.cpp
#   src/measmodel_pos.cpp
#   src/measmodel_vector.cpp
#   src/tracker_particle.cpp
#   src/tracker_kalman.cpp
#   src/detector_particle.cpp
# )

add_library(
  ${PROJECT_NAME} SHARED
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

target_include_directories(${PROJECT_NAME} PRIVATE ${BFL_INCLUDE_DIRS}/bfl include)
ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
  geometry_msgs
  message_filters
  people_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

target_link_libraries(
  ${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${BFL_LIBRARIES}
)

# list(
#   APPEND
#   people_tracker_FOUND_LIBRARIES
#   ${PROJECT_NAME}_FOUND_LIBRARIES
#   ${PROJECT_NAME}
# )

# ament_auto_add_executable(people_tracker src/people_tracking_node.cpp)

add_executable(people_tracker src/people_tracking_node.cpp)
target_include_directories(people_tracker PRIVATE ${BFL_INCLUDE_DIRS}/bfl include)
ament_target_dependencies(
  people_tracker
  rclcpp
  geometry_msgs
  message_filters
  people_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)
target_link_libraries(
  people_tracker
  ${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${BFL_LIBRARIES}
)

install(
  TARGETS people_tracker ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# ament_auto_package(INSTALL_TO_SHARE launch)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp
  geometry_msgs
  message_filters
  people_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros)

ament_package()
