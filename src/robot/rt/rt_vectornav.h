/*!
 * @file rt_vectornav.h
 * @brief VectorNav IMU communication
 */
#pragma once

#ifdef linux

#include <lcm/lcm-cpp.hpp>
#include "SimUtilities/IMUTypes.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "vn/sensors.h"

#ifdef __cplusplus
}
#endif

bool init_vectornav(VectorNavData* vd_data);

#endif
