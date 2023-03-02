#pragma once
#include "vec_types.h"

struct MasterConfig
{
    RobotType robot;
    bool      simulated      = false;
    bool      load_from_file = false;
};
