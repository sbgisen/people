cmake_minimum_required(VERSION 3.18)
project(orocos_bfl_vendor)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)

find_package(orocos_bfl 0.8 QUIET)

ament_vendor(${PROJECT_NAME}
  SATISFIED ${orocos_bfl_FOUND}
  VCS_URL https://github.com/orocos/orocos-bayesian-filtering.git
  VCS_VERSION master
  SOURCE_SUBDIR orocos_bfl
  VCS_TYPE git
)

ament_package(
  CONFIG_EXTRAS "orocos_bfl_vendor-extras.cmake"
)
