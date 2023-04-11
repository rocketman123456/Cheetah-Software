#include <iostream>

#include "rt_spi.h"

int main() {
    std::cout << "Hello World!" << std::endl;

    init_spi();

    spi_driver_run();

    return 0;
}