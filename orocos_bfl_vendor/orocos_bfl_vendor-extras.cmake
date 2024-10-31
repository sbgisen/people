
find_package(PkgConfig)
get_filename_component(INSTALL_DIR "${CMAKE_INSTALL_PREFIX}" DIRECTORY)
set(ENV{PKG_CONFIG_PATH} "$ENV{PKG_CONFIG_PATH}:${INSTALL_DIR}/orocos_bfl_vendor/opt/orocos_bfl_vendor/lib/pkgconfig")

pkg_check_modules(
  BFL
  REQUIRED
  orocos-bfl
)
