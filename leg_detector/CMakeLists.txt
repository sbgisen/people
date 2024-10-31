cmake_minimum_required(VERSION 3.8)
project(leg_detector)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(OpenCV REQUIRED)
link_directories(${BFL_LIBRARY_DIRS})
include_directories(${BFL_INCLUDE_DIRS})

# Look for Bullet
pkg_check_modules(BULLET bullet)

ament_auto_add_executable(
  ${PROJECT_NAME}
  src/laser_processor.cpp
  src/leg_detector.cpp
  src/calc_leg_features.cpp
)
target_link_libraries(
  ${PROJECT_NAME}
  ${BFL_LIBRARIES}
  ${BULLET_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
