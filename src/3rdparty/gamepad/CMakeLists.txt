cmake_minimum_required(VERSION 3.5)

include_directories("./")

file(GLOB_RECURSE sources "./*.cpp")
add_library(gamepad SHARED ${sources})
