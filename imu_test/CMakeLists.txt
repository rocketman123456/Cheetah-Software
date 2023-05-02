include_directories(${PROJECT_SOURCE_DIR})

add_executable(imt_test main.cpp)

target_link_libraries(imt_test libvnc)
