#include <iostream>

#include "rt_spi.h"

int main() {
    std::cout << "Hello World!" << std::endl;

    init_spi();

    spi_send_receive(&spi_command_drv, &spi_data_drv);

    return 0;
}