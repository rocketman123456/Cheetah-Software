cmake_minimum_required(VERSION 3.5)

project(CheetahCustom)

set(CMAKE_DISABLE_IN_SOURCE_BUILD ON)
set(CMAKE_DISABLE_SOURCE_CHANGES  OFF)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

if("${CMAKE_SOURCE_DIR}" STREQUAL "${CMAKE_BINARY_DIR}")
    message(SEND_ERROR "In-source builds are not allowed.")
endif()

set(CMAKE_COLOR_MAKEFILE   ON)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

add_executable(spi_test 
    main.cpp 
    hardware/rt_spi.cpp 
    bridge/hardware_bridge.cpp
    util/crc.cpp
)

#target_link_libraries(spi_test pthread)
