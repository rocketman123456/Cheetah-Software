#include "hardware/motor_spi.h"
#include "bridge/hardware_bridge.h"
#include "util/crc.h"

#include <cmath>
#include <iostream>
#include <unistd.h>

int main()
{
    HardwareBridge bridge;

    bridge.initialize();

    bridge.setJoint(LEFT, FRONT, ABAD, 0, 0, 5, 0, 0);
    bridge.setJoint(LEFT, FRONT, HIP, 0, 0, 5, 0, 0);
    bridge.setJoint(LEFT, FRONT, KNEE, 0, 0, 5, 0, 0);

    bridge.setJoint(LEFT, REAR, ABAD, 0, 0, 5, 0, 0);
    bridge.setJoint(LEFT, REAR, HIP, 0, 0, 5, 0, 0);
    bridge.setJoint(LEFT, REAR, KNEE, 0, 0, 5, 0, 0);

    bridge.setJoint(RIGHT, FRONT, ABAD, 0, 0, 5, 0, 0);
    bridge.setJoint(RIGHT, FRONT, HIP, 0, 0, 5, 0, 0);
    bridge.setJoint(RIGHT, FRONT, KNEE, 0, 0, 5, 0, 0);

    bridge.setJoint(RIGHT, REAR, ABAD, 0, 0, 5, 0, 0);
    bridge.setJoint(RIGHT, REAR, HIP, 0, 0, 5, 0, 0);
    bridge.setJoint(RIGHT, REAR, KNEE, 0, 0, 5, 0, 0);

    bridge.start();

    bridge.update();

    bridge.printInfo();

    bridge.finalize();

    return 0;
}
