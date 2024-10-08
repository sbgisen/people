cmake_minimum_required(VERSION 3.8)
project(people_msgs)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

file(
  GLOB MESSAGE_FILES
  RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
  msg/*.msg
)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  ${MESSAGE_FILES}
  DEPENDENCIES
  ${${PROJECT_NAME}_BUILD_DEPENDS}
)

ament_auto_package()
