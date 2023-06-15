#pragma once
#include "hardware/bridge/bridge_interface.h"
#include "hardware/motor_spi.h"

#include <cstdint>

const uint32_t LEFT  = 0;
const uint32_t RIGHT = 1;
const uint32_t FRONT = 0;
const uint32_t REAR  = 1;
const uint32_t ABAD  = 0;
const uint32_t HIP   = 1;
const uint32_t KNEE  = 2;

class MotorSpiBridge : public MotorBridgeInterface
{
    struct convert_motor
    {
        float sign;
        float offset;
    };

    struct convert_leg
    {
        convert_motor motor[3];
    };

    struct convert_spi
    {
        convert_leg leg[2];
    };

public:
    void initialize() override; // init communication
    void finalize() override;

    void update() override; // send cmd to motor and get state

    void publish() override;

    void start(); // enable motor
    void stop();  // disable motor

    void printInfo();

    // i: spi, LEFT, RIGHT, j: leg, FRONT, REAR, k:motor, ABAD, HIP, KNEE, motor cmd
    void setJoint(int i, int j, int k, float p_des, float v_des, float kp, float kd, float t_ff) override;

public:
    int m_spi_fd[2];

    spine_cmd_t   m_cmd[2];
    spine_state_t m_state[2];

    convert_spi m_converter[2];
};
