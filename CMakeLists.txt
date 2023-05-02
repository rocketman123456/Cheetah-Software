cmake_minimum_required(VERSION 3.16)
project(motor_control_demo LANGUAGES CXX)

set(CMAKE_DISABLE_IN_SOURCE_BUILD ON)
set(CMAKE_DISABLE_SOURCE_CHANGES OFF)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

if("${CMAKE_SOURCE_DIR}" STREQUAL "${CMAKE_BINARY_DIR}")
    message(SEND_ERROR "In-source builds are not allowed.")
endif()

set(CMAKE_COLOR_MAKEFILE ON)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
    add_compile_options(-Wall -Wextra)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
)

# find dependencies
find_package(ament_cmake REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
    find_package(${Dependency} REQUIRED)
endforeach()

# # COMPILE
add_library(
    motor_control_demo
    SHARED
    src/motor_interface.cpp
)

target_compile_features(motor_control_demo PUBLIC cxx_std_17)

target_include_directories(motor_control_demo PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/motor_control_demo>
)

ament_target_dependencies(
    motor_control_demo PUBLIC
    ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "ROS2_CONTROL_DEMO_EXAMPLE_2_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface motor_control_demo.xml)

# INSTALL
install(
    DIRECTORY include/
    DESTINATION include/motor_control_demo
)
# install(
#     DIRECTORY description/launch description/ros2_control description/urdf description/rviz
#     DESTINATION share/motor_control_demo
# )
# install(
#     DIRECTORY bringup/launch bringup/config
#     DESTINATION share/motor_control_demo
# )
# install(TARGETS motor_control_demo
#     EXPORT export_motor_control_demo
#     ARCHIVE DESTINATION lib
#     LIBRARY DESTINATION lib
#     RUNTIME DESTINATION bin
# )

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
endif()

# # EXPORTS
ament_export_targets(motor_control_demo HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
