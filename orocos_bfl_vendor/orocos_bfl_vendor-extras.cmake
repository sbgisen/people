
find_package(PkgConfig)
set(ENV{PKG_CONFIG_PATH} "$ENV{PKG_CONFIG_PATH}:${orocos_bfl_vendor_DIR}/../../../opt/orocos_bfl_vendor/lib/pkgconfig")

pkg_check_modules(
  BFL
  REQUIRED
  orocos-bfl
)
