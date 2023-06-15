#pragma once

class BridgeInterface
{
public:
    virtual void initialize() = 0;
    virtual void finalize()   = 0;
    virtual void update()     = 0;
    virtual void publish()    = 0;
};

class MotorBridgeInterface : public BridgeInterface
{
public:
    // i: spi, LEFT, RIGHT, j: leg, FRONT, REAR, k:motor, ABAD, HIP, KNEE, motor cmd
    virtual void setJoint(int i, int j, int k, float p_des, float v_des, float kp, float kd, float t_ff) = 0;
};

class IMUBridgeInterface : public BridgeInterface
{};
