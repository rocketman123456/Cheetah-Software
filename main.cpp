#include "hardware/motor_spi.h"
#include "bridge/motor_spi_bridge.h"
#include "util/crc.h"
#include "util/timer.h"

#include <cmath>
#include <iostream>
#include <unistd.h>

MotorSpiBridge bridge;

float angle = 0;

void simple_control()
{
    angle += 0.1;

    bridge.setJoint(LEFT, FRONT, ABAD, 0, 0, 5.0, 0, 0);
    bridge.setJoint(LEFT, REAR, ABAD, 0, 0, 5.0, 0, 0);
    bridge.setJoint(RIGHT, FRONT, ABAD, 0, 0, 5.0, 0, 0);
    bridge.setJoint(RIGHT, REAR, ABAD, 0, 0, 5.0, 0, 0);
    bridge.update();
    bridge.printInfo();
}

int main()
{
    //prefaultStack();
    //setupScheduler();
    //create_lookup_table();

    bridge.initialize();

    // bridge.start();

    Timer timer;

    std::cout << "--- start period timer ----" << std::endl;
	timer.start(100, std::bind(simple_control));
	std::this_thread::sleep_for(std::chrono::milliseconds(5000));
	timer.stop();
	std::cout << "--- stop period timer ----" << std::endl;

    bridge.finalize();

    return 0;
}
