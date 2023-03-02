include_directories(${PROJECT_SOURCE_DIR})

add_executable(joint_ctrl main.cpp)

target_link_directories(joint_ctrl PUBLIC
    /usr/local/lib
)

target_link_libraries(joint_ctrl 
    ma27 ma57 hsl_ma77 hsl_ma86 coinhsl ipopt
)
