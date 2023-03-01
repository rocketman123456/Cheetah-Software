cmake_minimum_required(VERSION 3.5)
set(CMAKE_DISABLE_IN_SOURCE_BUILD ON)
set(CMAKE_DISABLE_SOURCE_CHANGES  ON)

if ("${CMAKE_SOURCE_DIR}" STREQUAL "${CMAKE_BINARY_DIR}")
    message(SEND_ERROR "In-source builds are not allowed.")
endif ()

set(CMAKE_COLOR_MAKEFILE   ON)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
#set(CMAKE_VERBOSE_MAKEFILE ON)

option(MINI_CHEETAH_BUILD "use compiler flags for mini cheetah computer" OFF)
# set(BUILD_TYPE_RELEASE TRUE)

include_directories(${CMAKE_BINARY_DIR})
include_directories("common/include")
include_directories("robot/include")

