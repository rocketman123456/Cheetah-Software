/*!
 * @file rt_spi.h
 * @brief SPI communication to spine board
 */

#include "rt_spi.h"

#include <byteswap.h>
#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <string>

#include <linux/spi/spidev.h>

unsigned char spi_mode          = SPI_MODE_0; // 时钟极性0,时钟相位0
unsigned char spi_bits_per_word = 8;          // 每个字8字节
unsigned int  spi_speed         = 6000000;    // 6M
uint8_t       lsb               = 0x01;

int spi_1_fd = -1;
int spi_2_fd = -1;

/*!
 * Compute SPI message checksum
 * @param data : input
 * @param len : length (in 32-bit words)
 * @return
 */
uint32_t xor_checksum(uint32_t* data, size_t len)
{
    uint32_t t = 0;
    for (size_t i = 0; i < len; i++)
        t = t ^ data[i];
    return t;
}

/*!
 * Open SPI device
 * 启动spi设备
 */
int spi_open()
{
    // 打开spi
    printf("[RT SPI] Open\n");
    int rv = 0; // receive value
    // 通过打开文件的方式打开设备吗? fd=file device
    spi_1_fd = open("/dev/spidev0.0", O_RDWR); // O_RDWR :文件可读可写
    if (spi_1_fd < 0)
        perror("[ERROR] Couldn't open spidev 2.0");
    spi_2_fd = open("/dev/spidev0.1", O_RDWR);
    if (spi_2_fd < 0)
        perror("[ERROR] Couldn't open spidev 2.1");
    // ioctl:Input output control.设置一些io信息
    // spi_mode 设置极性与相位
    rv = ioctl(spi_1_fd, SPI_IOC_WR_MODE, &spi_mode);
    if (rv < 0)
        perror("[ERROR] ioctl spi_ioc_wr_mode (1)");
    rv = ioctl(spi_2_fd, SPI_IOC_WR_MODE, &spi_mode);
    if (rv < 0)
        perror("[ERROR] ioctl spi_ioc_wr_mode (2)");

    rv = ioctl(spi_1_fd, SPI_IOC_RD_MODE, &spi_mode);
    if (rv < 0)
        perror("[ERROR] ioctl spi_ioc_rd_mode (1)");
    rv = ioctl(spi_2_fd, SPI_IOC_RD_MODE, &spi_mode);
    if (rv < 0)
        perror("[ERROR] ioctl spi_ioc_rd_mode (2)");
    // 设置每个字有几个字节
    rv = ioctl(spi_1_fd, SPI_IOC_WR_BITS_PER_WORD, &spi_bits_per_word);
    if (rv < 0)
        perror("[ERROR] ioctl spi_ioc_wr_bits_per_word (1)");
    rv = ioctl(spi_2_fd, SPI_IOC_WR_BITS_PER_WORD, &spi_bits_per_word);
    if (rv < 0)
        perror("[ERROR] ioctl spi_ioc_wr_bits_per_word (2)");

    rv = ioctl(spi_1_fd, SPI_IOC_RD_BITS_PER_WORD, &spi_bits_per_word);
    if (rv < 0)
        perror("[ERROR] ioctl spi_ioc_rd_bits_per_word (1)");
    rv = ioctl(spi_2_fd, SPI_IOC_RD_BITS_PER_WORD, &spi_bits_per_word);
    if (rv < 0)
        perror("[ERROR] ioctl spi_ioc_rd_bits_per_word (2)");
    // 设置spi通信速度 6M
    rv = ioctl(spi_1_fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed);
    if (rv < 0)
        perror("[ERROR] ioctl spi_ioc_wr_max_speed_hz (1)");
    rv = ioctl(spi_2_fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed);
    if (rv < 0)
        perror("[ERROR] ioctl spi_ioc_wr_max_speed_hz (2)");

    rv = ioctl(spi_1_fd, SPI_IOC_RD_MAX_SPEED_HZ, &spi_speed);
    if (rv < 0)
        perror("[ERROR] ioctl spi_ioc_rd_max_speed_hz (1)");
    rv = ioctl(spi_2_fd, SPI_IOC_RD_MAX_SPEED_HZ, &spi_speed);
    if (rv < 0)
        perror("[ERROR] ioctl spi_ioc_rd_max_speed_hz (2)");
    // 采用lsb模式
    rv = ioctl(spi_1_fd, SPI_IOC_RD_LSB_FIRST, &lsb);
    if (rv < 0)
        perror("[ERROR] ioctl spi_ioc_rd_lsb_first (1)");
    rv = ioctl(spi_2_fd, SPI_IOC_RD_LSB_FIRST, &lsb);
    if (rv < 0)
        perror("[ERROR] ioctl spi_ioc_rd_lsb_first (2)");

    return rv;
}

void spi_close()
{
    close(spi_1_fd);
    close(spi_2_fd);
}

std::string hex2str(const uint8_t* str, int len)
{
    static const char hexTable[17] = "0123456789ABCDEF";

    std::string result;
    for (int i = 0; i < len; ++i)
    {
        result += "0x";
        result += hexTable[(unsigned char)str[i] / 16];
        result += hexTable[(unsigned char)str[i] % 16];
        result += " ";
    }
    return result;
}

int transfer(int fd, uint8_t const *tx, uint8_t const *rx, size_t len)
{
    struct spi_ioc_transfer tr = {
        .tx_buf = (uint64_t)tx,
        .rx_buf = (uint64_t)rx,
        .len = len,
        .speed_hz = spi_speed,
        .delay_usecs = 0,
        .bits_per_word = spi_bits_per_word,
        .cs_change = 1,
    };

    int ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
    if (ret < 1)
        printf("can't send spi message\n");

    return ret;
}
