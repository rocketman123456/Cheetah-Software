#include "rt_spi.h"

// #include "PeriodicTask.h"

#include <cmath>
#include <iostream>
#include <unistd.h>

int main()
{
    spi_open();

    uint8_t tx[16];
    uint8_t rx[16];

    for(int i = 0; i < 16; ++i)
    {
        tx[i] = i;
    }

    transfer(spi_1_fd, tx, rx, 16);
    auto str = hex2str(rx, 16);
    std::cout << "[SPI1]" << str << endl;

    transfer(spi_2_fd, tx, rx, 16);
    auto str = hex2str(rx, 16);
    std::cout << "[SPI2]" << str << endl;

    spi_close();

    return 0;
}
