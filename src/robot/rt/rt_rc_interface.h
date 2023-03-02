/**
 * @file rt_rc_interface.h
 *
 */
#pragma once

class rc_control_settings
{
public:
    double mode;
    double p_des[2];         // (x, y) -1 ~ 1
    double height_variation; // -1 ~ 1
    double v_des[3];         // -1 ~ 1 * (scale 0.5 ~ 1.5)
    double rpy_des[3];       // -1 ~ 1
    double omega_des[3];     // -1 ~ 1
    double variable[3];
    double step_height;
};

namespace RC_mode
{
    constexpr int OFF          = 0;
    constexpr int STAND_UP     = 1;
    constexpr int READY        = 2;
    constexpr int QP_STAND     = 3;
    constexpr int BACKFLIP_PRE = 4;
    constexpr int BACKFLIP     = 5;
    constexpr int VISION       = 6;

    constexpr int LOCOMOTION     = 11;
    constexpr int RECOVERY_STAND = 12;
    constexpr int FRONT_JUMP     = 13;
    constexpr int RC_SIT_DOWN    = 14;

    // Experiment Mode
    constexpr int TWO_LEG_STANCE_PRE = 20;
    constexpr int TWO_LEG_STANCE     = 21;

    constexpr int WAIT = 31;
}; // namespace RC_mode

void sbus_packet_complete();
void sbus_packet_complete_at9s();

void get_rc_control_settings(void* settings);
// void get_rc_channels(void* settings);

void* v_memcpy(void* dest, volatile void* src, size_t n);

float deadband(float command, float deadbandRegion, float minVal, float maxVal);
