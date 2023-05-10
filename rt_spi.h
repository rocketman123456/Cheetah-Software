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
#include <unistd.h> //Needed for SPI port

#define K_EXPECTED_COMMAND_SIZE 256
#define K_EXPECTED_DATA_SIZE 164
#define K_KNEE_OFFSET_POS 4.35f

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte) \
    (byte & 0x80 ? '1' : '0'), (byte & 0x40 ? '1' : '0'), \
    (byte & 0x20 ? '1' : '0'), (byte & 0x10 ? '1' : '0'), \
    (byte & 0x08 ? '1' : '0'), (byte & 0x04 ? '1' : '0'), \
    (byte & 0x02 ? '1' : '0'), (byte & 0x01 ? '1' : '0')

#pragma pack(1)
/*!
 * SPI command message
 */
typedef struct // 132
{
    float q_des_abad[2] = {0};
    float q_des_hip[2]  = {0};
    float q_des_knee[2] = {0};

    float qd_des_abad[2] = {0};
    float qd_des_hip[2]  = {0};
    float qd_des_knee[2] = {0};

    float kp_abad[2] = {0};
    float kp_hip[2]  = {0};
    float kp_knee[2] = {0};

    float kd_abad[2] = {0};
    float kd_hip[2]  = {0};
    float kd_knee[2] = {0};

    float tau_abad_ff[2] = {0};
    float tau_hip_ff[2]  = {0};
    float tau_knee_ff[2] = {0};

    int32_t flags[2] = {0};
    int32_t checksum;
} spine_cmd_t;

typedef struct // 84
{
    float q_abad[2] = {0};
    float q_hip[2]  = {0};
    float q_knee[2] = {0};

    float qd_abad[2] = {0};
    float qd_hip[2]  = {0};
    float qd_knee[2] = {0};

    float tau_abad[2] = {0};
    float tau_hip[2]  = {0};
    float tau_knee[2] = {0};

    int32_t flags[2] = {0};
    int32_t checksum = 0;
} spine_data_t;

typedef struct // 256
{
    float q_des_abad[4] = {0};
    float q_des_hip[4]  = {0};
    float q_des_knee[4] = {0};

    float qd_des_abad[4] = {0};
    float qd_des_hip[4]  = {0};
    float qd_des_knee[4] = {0};

    float kp_abad[4] = {0};
    float kp_hip[4]  = {0};
    float kp_knee[4] = {0};

    float kd_abad[4] = {0};
    float kd_hip[4]  = {0};
    float kd_knee[4] = {0};

    float tau_abad_ff[4] = {0};
    float tau_hip_ff[4]  = {0};
    float tau_knee_ff[4] = {0};

    int32_t flags[4] = {0};
} spi_command_t;

typedef struct // 164
{
    float q_abad[4] = {0};
    float q_hip[4]  = {0};
    float q_knee[4] = {0};

    float qd_abad[4] = {0};
    float qd_hip[4]  = {0};
    float qd_knee[4] = {0};

    float tau_abad[4] = {0};
    float tau_hip[4]  = {0};
    float tau_knee[4] = {0};

    int32_t flags[4] = {0};

    int32_t spi_driver_status = 0;
} spi_data_t;

typedef struct // 48
{
    float tau_abad[4];
    float tau_hip[4];
    float tau_knee[4];
} spi_torque_t;
#pragma pack()

extern spi_command_t spi_command_drv;
extern spi_data_t    spi_data_drv;
extern spi_torque_t  spi_torque;

void init_spi();

int  spi_open();
void spi_close();

void spi_to_spine(spi_command_t* cmd, spine_cmd_t* spine_cmd, int leg_0);
void spine_to_spi(spi_data_t* data, spine_data_t* spine_data, int leg_0);

void spi_send_receive(spi_command_t* command, spi_data_t* data);
void spi_driver_run();

spi_data_t*    get_spi_data();
spi_command_t* get_spi_command();
