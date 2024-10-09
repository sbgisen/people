cmake_minimum_required(VERSION 3.8)
project(leg_detector)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_geometry REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(message_filters REQUIRED)
find_package(people_msgs REQUIRED)
find_package(people_tracking_filter REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
# look for bfl (Bayesian Filtering Library)
find_package(PkgConfig REQUIRED)
pkg_check_modules(
  BFL
  REQUIRED
  orocos-bfl
)
link_directories(${BFL_LIBRARY_DIRS})
find_package(OpenCV REQUIRED)

# Look for Bullet
pkg_check_modules(BULLET bullet)

# find_package(catkin REQUIRED COMPONENTS
#         dynamic_reconfigure
#         geometry_msgs
#         image_geometry
#         laser_geometry
#         message_filters
#         people_msgs
#         people_tracking_filter
#         roscpp
#         sensor_msgs
#         std_msgs
#         std_srvs
#         tf
#         tf2_geometry_msgs
#         tf2_ros
#         visualization_msgs
# )

# catkin_package(
#     INCLUDE_DIRS include
#     CATKIN_DEPENDS
#         dynamic_reconfigure
#         geometry_msgs
#         message_filters
#         people_msgs
#         people_tracking_filter
#         roscpp
#         sensor_msgs
#         std_msgs
#         tf
#         visualization_msgs
# )

# include_directories(
#   include ${catkin_INCLUDE_DIRS} ${BFL_INCLUDE_DIRS} ${BULLET_INCLUDE_DIRS}
# )

# add_executable(leg_detector
#                src/laser_processor.cpp
#                src/leg_detector.cpp
#                src/calc_leg_features.cpp)

# add_dependencies(leg_detector ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# target_link_libraries(leg_detector
#    ${catkin_LIBRARIES} ${BFL_LIBRARIES} ${BULLET_LIBRARIES}
# )

include_directories(
  include
  ${BFL_INCLUDE_DIRS}/bfl
  ${BULLET_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
add_executable(
  ${PROJECT_NAME}
  src/laser_processor.cpp
  src/leg_detector.cpp
  src/calc_leg_features.cpp
)
ament_target_dependencies(
  ${PROJECT_NAME}
  geometry_msgs
  image_geometry
  laser_geometry
  message_filters
  people_msgs
  people_tracking_filter
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)
target_link_libraries(
  ${PROJECT_NAME}
  ${BFL_LIBRARIES}
  ${BULLET_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_dependencies(
  geometry_msgs
  image_geometry
  laser_geometry
  message_filters
  people_msgs
  people_tracking_filter
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

ament_package()
