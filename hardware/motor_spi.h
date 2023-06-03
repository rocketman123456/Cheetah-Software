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

#pragma pack(1)

// 24 byte
typedef struct
{
    float p_des;
    float v_des;
    float kp;
    float kd;
    float t_ff;
} motor_cmd_t;

// 64 byte
typedef struct
{
    motor_cmd_t motor[3];
    int32_t flag;
} leg_cmd_t;

// 132 byte
typedef struct
{
    leg_cmd_t leg[2];
    uint32_t crc;
} spine_cmd_t;

// --------------------------------------
// --------------------------------------
// --------------------------------------
// --------------------------------------
// --------------------------------------

// 12 byte
typedef struct 
{
    float p;
    float v;
    float t;
} motor_data_t;

// 40 byte
typedef struct
{
    motor_data_t motor[3];
    int32_t flag;
} leg_state_t;

// 84 byte
typedef struct
{
    leg_state_t leg[2];
    uint32_t crc;
} spine_state_t;

#pragma pack()
