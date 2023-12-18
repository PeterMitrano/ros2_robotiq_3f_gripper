# See https://docs.ros.org/en/foxy/How-To-Guides/Ament-CMake-Documentation.html

cmake_minimum_required(VERSION 3.8)
project(robotiq_3f_controllers VERSION 0.0.1 LANGUAGES CXX)

# This module provides installation directories as per the GNU coding standards.
include(GNUInstallDirs)

set(CMAKE_EXPORT_COMPILE_COMMANDS true)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(robotiq_3f_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(std_srvs REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  robotiq_3f_msgs
  rclcpp
  rclcpp_action
  realtime_tools
  std_srvs
)

# Robotiq3f controller library.

add_library(
  robotiq_3f_controllers
  SHARED
  src/robotiq_3f_controller.cpp
  include/robotiq_3f_controllers/robotiq_3f_controller.hpp
)
target_include_directories(
  robotiq_3f_controllers
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
  robotiq_3f_controllers
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

###############################################################################
# PLUGINS

pluginlib_export_plugin_description_file(controller_interface controller_plugins.xml)

###############################################################################
# EXPORTS

# This is necessary to allow this library’s clients to use the syntax
# target_link_libraries(client <pkg>::<target_name>)
# target_link_libraries(client stepit_hardware::stepit_hardware)
# Without this a client cannot find this library.
# It can take an arbitrary list of targets named as EXPORT in an install call.
ament_export_targets(
  robotiq_3f_controllers_targets # Must match the EXPORT label below in the install section.
)
# Help downstream packages to find transitive dependencies i.e. export all
# dependencies required by a package to use this library.
# When a package calls find_package(robotiq_3f_driver), CMake looks for a file
# called robotiq_3f_driverConfig.cmake which sets up everything another project
# would need to depend on this one.
ament_export_dependencies(
    ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
# Tell downstream packages where to find our headers.
ament_export_include_directories(
    include
)
# Tell downstream packages our libraries to link against.
ament_export_libraries(
  robotiq_3f_controllers
)

###############################################################################
# INSTALL

# Install all files of the include folder into the give destination.
install(
    DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# Install our library.
install(
  TARGETS robotiq_3f_controllers
  EXPORT robotiq_3f_controllers_targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}      # lib
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}      # lib
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}      # bin
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} # include
)

ament_package()
