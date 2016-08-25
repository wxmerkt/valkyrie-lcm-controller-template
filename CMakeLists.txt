cmake_minimum_required(VERSION 2.8.12)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(POD_NAME lcm_controller_template)
include(cmake/pods.cmake)

find_package(PkgConfig REQUIRED)
pkg_check_modules(lcm lcm)

add_executable(lcm-controller-template lcm_controller_template.cc)
target_link_libraries(lcm-controller-template )
pods_use_pkg_config_packages(lcm-controller-template lcm bot2-core lcmtypes_drc_lcmtypes lcmtypes_bot2-core)
