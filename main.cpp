#include "rt_spi.h"

// #include "PeriodicTask.h"

#include <cmath>
#include <iostream>
#include <unistd.h>

class SpiLoop
{
public:
    void runSpi()
    {
        spi_driver_run();
    }
};

int main()
{
    std::cout << "Hello World!" << std::endl;

    init_spi();

    spi_open();

    spi_command_drv.flags[0] = 1;
    spi_command_drv.flags[1] = 1;
    spi_command_drv.flags[2] = 1;
    spi_command_drv.flags[3] = 1;

    spi_command_drv.kp_abad[0] = 10;
    spi_command_drv.kp_abad[1] = 10;
    spi_command_drv.kp_abad[2] = 10;
    spi_command_drv.kp_abad[3] = 10;

    spi_command_drv.kd_abad[0] = 0.5;
    spi_command_drv.kd_abad[1] = 0.5;
    spi_command_drv.kd_abad[2] = 0.5;
    spi_command_drv.kd_abad[3] = 0.5;

    SpiLoop loop;
    loop.runSpi();

    spi_close();

    return 0;
}
