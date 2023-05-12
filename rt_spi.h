/*!
 * @file rt_spi.h
 * @brief SPI communication to spine board
 */

#pragma once

#include <fcntl.h>     //Needed for SPI port
#include <sys/ioctl.h> //Needed for SPI port

// incredibly obscure bug in SPI_IOC_MESSAGE macro is fixed by this
#ifdef __cplusplus /* If this is a C++ compiler, use C linkage */
extern "C"
{
#endif

#include <linux/spi/spidev.h>

#ifdef __cplusplus /* If this is a C++ compiler, use C linkage */
}
#endif

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <unistd.h> //Needed for SPI port

int  spi_open(int& fd, const char* name);
void spi_close(int fd);

int transfer(int fd, uint8_t const *tx, uint8_t const *rx, size_t len);

std::string hex2str(const uint8_t* str, int len);

extern int spi_1_fd;
extern int spi_2_fd;
