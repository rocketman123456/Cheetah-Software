#include "rt_spi.h"

// #include "PeriodicTask.h"

#include <cmath>
#include <iostream>
#include <unistd.h>

int spi_1_fd = -1;
int spi_2_fd = -1;

const char* name1 = "/dev/spidev0.0";
const char* name2 = "/dev/spidev0.1";

int main()
{
    spi_open(spi_1_fd, name1);

    uint8_t tx[16];
    uint8_t rx_1[16];
    uint8_t rx_2[16];

    for (int i = 0; i < 16; ++i)
    {
        tx[i] = i;
    }

    int rv = transfer(spi_1_fd, tx, rx_1, 16);
    (void)rv;
    auto str = hex2str(rx_1, 16);
    std::cout << name1 << ": "<< str << std::endl;

    spi_close(spi_1_fd);

    return 0;
}
