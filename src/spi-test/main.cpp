#include <iostream>

#include "rt_spi.h"

int main() {
    std::cout << "Hello World!" << std::endl;

    init_spi();

    spi_command_drv.flags[0] = 1;
    spi_command_drv.flags[1] = 1;
    spi_command_drv.flags[2] = 1;
    spi_command_drv.flags[3] = 1;

    spi_driver_run();

    return 0;
}