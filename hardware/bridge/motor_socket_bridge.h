#pragma once
#include "hardware/bridge/bridge_interface.h"
#include "hardware/motor_socketcan.h"

class MotorSocketBridge : public MotorBridgeInterface
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

public:
    // init communication
    void initialize() override;
    void finalize() override;

    // send cmd to motor and get state
    void update() override;

    void publish() override;

    void setJoint(int i, int j, int k, float p_des, float v_des, float kp, float kd, float t_ff) override;

private:
    int m_spi_fd[4];

    leg_cmd_t   m_cmd[4];
    leg_state_t m_state[4];

    convert_leg m_converter[4];
};
