/*!
 * @file rt_vectornav.h
 * @brief VectorNav IMU communication
 */
#pragma once

#include "IMUTypes.h"

#ifdef __cplusplus
extern "C"
{
#endif

#include "vn/sensors.h"

#ifdef __cplusplus
}
#endif

class vectornav_lcmt
{
public:
    float q[4];
    float w[3];
    float a[3];
}

bool init_vectornav(VectorNavData* vd_data);
