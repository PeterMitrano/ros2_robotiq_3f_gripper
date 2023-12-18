# See https://docs.ros.org/en/foxy/How-To-Guides/Ament-CMake-Documentation.html

cmake_minimum_required(VERSION 3.8)
project(robotiq_3f_driver VERSION 0.0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)

# This module provides installation directories as per the GNU coding standards.
include(GNUInstallDirs)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(transmission_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(serial REQUIRED)
find_package(robotiq_3f_transmission_plugins REQUIRED)
find_package(robotiq_3f_interfaces REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
        ament_cmake
        hardware_interface
        pluginlib
        rclcpp
        rclcpp_lifecycle
        robotiq_3f_transmission_plugins
        robotiq_3f_interfaces
        serial
)

# Robotiq 3f driver library.

add_library(
        robotiq_3f_driver
        SHARED
        include/robotiq_3f_driver/fake/fake_driver.hpp
        include/robotiq_3f_driver/crc_utils.hpp
        include/robotiq_3f_driver/data_utils.hpp
        include/robotiq_3f_driver/default_driver.hpp
        include/robotiq_3f_driver/default_driver_factory.hpp
        include/robotiq_3f_driver/default_driver_utils.hpp
        include/robotiq_3f_driver/default_serial.hpp
        include/robotiq_3f_driver/default_serial_factory.hpp
        include/robotiq_3f_driver/driver.hpp
        include/robotiq_3f_driver/driver_exception.hpp
        include/robotiq_3f_driver/driver_factory.hpp
        include/robotiq_3f_driver/robotiq_3f_gripper_hardware_interface.hpp
        include/robotiq_3f_driver/hardware_interface_utils.hpp
        include/robotiq_3f_driver/serial.hpp
        include/robotiq_3f_driver/serial_factory.hpp
        include/robotiq_3f_driver/visibility_control.hpp
        src/robotiq_3f_gripper_hardware_interface.cpp
        src/crc_utils.cpp
        src/data_utils.cpp
        src/default_driver.cpp
        src/default_driver_factory.cpp
        src/default_driver_utils.cpp
        src/fake/fake_driver.cpp
        src/default_serial.cpp
        src/default_serial_factory.cpp
        src/hardware_interface_utils.cpp
)
target_link_libraries(robotiq_3f_driver atomic)
target_include_directories(
        robotiq_3f_driver
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
        robotiq_3f_driver
        ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

###############################################################################
# PLUGINS

pluginlib_export_plugin_description_file(hardware_interface hardware_interface_plugin.xml)

###############################################################################
# EXPORTS

# This is necessary to allow this library’s clients to use the syntax
# target_link_libraries(client <pkg>::<target_name>)
# target_link_libraries(client stepit_hardware::stepit_hardware)
# Without this a client cannot find this library.
# It can take an arbitrary list of targets named as EXPORT in an install call.
ament_export_targets(
        robotiq_3f_driver_targets # Must match the EXPORT label below in the install section.
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
        robotiq_3f_driver
)

###############################################################################
# Main program to demo the driver on real hardware

add_executable(
        robotiq_3f_driver_node
        src/robotiq_3f_driver_node.cpp
)
target_link_libraries(
        robotiq_3f_driver_node
        robotiq_3f_driver
        robotiq_3f_transmission_plugins::individual_control_transmission
)

###############################################################################
# INSTALL

# Install all files of the include folder into the give destination.
install(
        DIRECTORY include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}          # include
)

# Install our library.
install(
        TARGETS robotiq_3f_driver robotiq_3f_driver_node
        EXPORT robotiq_3f_driver_targets
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}      # lib
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}      # lib
        RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}
        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} # include
)

###############################################################################
# TESTS

# CTest module automatically creates a BUILD_TESTING option that selects
# whether to enable testing support (ON by default).
include(CTest)
if (BUILD_TESTING)
    add_subdirectory(tests)
endif ()

###############################################################################
# LINTERS

add_custom_target(format
        COMMAND clang-format -i `git ls-files *.hpp *.cpp`
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
add_custom_target(tidy
        COMMAND clang-tidy -p ${CMAKE_BINARY_DIR} `git ls-files *.cpp`
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

ament_package()
