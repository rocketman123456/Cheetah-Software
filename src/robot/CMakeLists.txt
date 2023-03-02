# add_library(robot_rt
#     rt/rt_ethercat.cpp
#     rt/rt_rc_interface.cpp
#     rt/rt_serial.cpp
#     rt/rt_sbus.cpp
#     rt/rt_spi.cpp
#     rt/rt_vectornav.cpp
# )

add_library(robot
    HardwareBridge.cpp
    RobotController.cpp
    RobotRunner.cpp
)