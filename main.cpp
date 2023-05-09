#include "rt_spi.h"

#include <cmath>
#include <iostream>
#include <unistd.h>

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

    int i = 0;
    while (i < 10)
    {
        ++i;
        spi_driver_run();
        sleep(1);
    }

    spi_close();

    return 0;
}
