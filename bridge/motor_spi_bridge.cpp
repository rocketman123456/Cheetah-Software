#include "bridge/motor_spi_bridge.h"
#include "hardware/motor_spi.h"
#include "hardware/bcm2835.h"

#define PIN_SPI_0 RPI_BPLUS_GPIO_J8_24
#define PIN_SPI_1 RPI_BPLUS_GPIO_J8_26

#include "rt/rt_util.h"
#include "util/crc.h"

#include <iostream>
#include <chrono>
#include <thread>
#define _USE_MATH_DEFINES
#include <math.h>

#include "pigpio.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

const char* name[] = {"/dev/spidev0.0", "/dev/spidev0.1"};

const int spi_pin[] = {PIN_SPI_0, PIN_SPI_1};
// const int spi_pin[] = {BCM2835_SPI_CS0, BCM2835_SPI_CS1};

const uint32_t spi_count = 2;
const uint32_t leg_count = 2;
const uint32_t motor_count = 3;

// init communication
void MotorSpiBridge::initialize()
{
    // bcm2835_init();
    // bcm2835_spi_begin();
    // bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_LSBFIRST);
    // bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_64);
    // bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);

    // bcm2835_spi_chipSelect(BCM2835_SPI_CS0);
    // bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);
    // bcm2835_spi_chipSelect(BCM2835_SPI_CS1);
    // bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS1, LOW);

    // spi_open(m_spi_fd[0], name[0]);
    // spi_open(m_spi_fd[1], name[1]);

    m_spi_fd[0] = spiOpen(0, 6000000, 0);
    m_spi_fd[1] = spiOpen(1, 6000000, 0);

    // bcm2835_gpio_fsel(spi_pin[0], BCM2835_GPIO_FSEL_OUTP);
    // bcm2835_gpio_fsel(spi_pin[1], BCM2835_GPIO_FSEL_OUTP);
    // bcm2835_gpio_write(spi_pin[0], HIGH);
    // bcm2835_gpio_write(spi_pin[1], HIGH);

    float angle1 = 0;//M_PI / 2.0;
    float angle2 = 0;//M_PI - 26.0 * M_PI / 180.0;

    m_converter[LEFT].leg[FRONT].motor[ABAD].sign = -1.0;
    m_converter[LEFT].leg[FRONT].motor[ABAD].offset = 0;
    m_converter[LEFT].leg[FRONT].motor[HIP].sign = -1.0;
    m_converter[LEFT].leg[FRONT].motor[HIP].offset = -angle1;
    m_converter[LEFT].leg[FRONT].motor[KNEE].sign = 1.0;
    m_converter[LEFT].leg[FRONT].motor[KNEE].offset = -angle2;

    m_converter[LEFT].leg[REAR].motor[ABAD].sign = 1.0;
    m_converter[LEFT].leg[REAR].motor[ABAD].offset = 0;
    m_converter[LEFT].leg[REAR].motor[HIP].sign = -1.0;
    m_converter[LEFT].leg[REAR].motor[HIP].offset = angle1;
    m_converter[LEFT].leg[REAR].motor[KNEE].sign = 1.0;
    m_converter[LEFT].leg[REAR].motor[KNEE].offset = angle2;

    m_converter[RIGHT].leg[FRONT].motor[ABAD].sign = -1.0;
    m_converter[RIGHT].leg[FRONT].motor[ABAD].offset = 0;
    m_converter[RIGHT].leg[FRONT].motor[HIP].sign = 1.0;
    m_converter[RIGHT].leg[FRONT].motor[HIP].offset = -angle1;
    m_converter[RIGHT].leg[FRONT].motor[KNEE].sign = -1.0;
    m_converter[RIGHT].leg[FRONT].motor[KNEE].offset = -angle2;

    m_converter[RIGHT].leg[REAR].motor[ABAD].sign = 1.0;
    m_converter[RIGHT].leg[REAR].motor[ABAD].offset = 0;
    m_converter[RIGHT].leg[REAR].motor[HIP].sign = 1.0;
    m_converter[RIGHT].leg[REAR].motor[HIP].offset = angle1;
    m_converter[RIGHT].leg[REAR].motor[KNEE].sign = -1.0;
    m_converter[RIGHT].leg[REAR].motor[KNEE].offset = angle2;

    float default_kp = 0.4;

    float angle1_d = 0;//M_PI / 2.0;
    float angle2_d = 0;//M_PI / 1.5;

    // init a very soft behavior
    setJoint(LEFT, FRONT, ABAD, 0, 0, default_kp, 0, 0);
    setJoint(LEFT, FRONT, HIP, angle1_d, 0, default_kp, 0, 0);
    setJoint(LEFT, FRONT, KNEE, -angle2_d, 0, default_kp, 0, 0);

    setJoint(LEFT, REAR, ABAD, 0, 0, default_kp, 0, 0);
    setJoint(LEFT, REAR, HIP, angle1_d, 0, default_kp, 0, 0);
    setJoint(LEFT, REAR, KNEE, -angle2_d, 0, default_kp, 0, 0);

    setJoint(RIGHT, FRONT, ABAD, 0, 0, default_kp, 0, 0);
    setJoint(RIGHT, FRONT, HIP, angle1_d, 0, default_kp, 0, 0);
    setJoint(RIGHT, FRONT, KNEE, -angle2_d, 0, default_kp, 0, 0);

    setJoint(RIGHT, REAR, ABAD, 0, 0, default_kp, 0, 0);
    setJoint(RIGHT, REAR, HIP, angle1_d, 0, default_kp, 0, 0);
    setJoint(RIGHT, REAR, KNEE, -angle2_d, 0, default_kp, 0, 0);
}

void MotorSpiBridge::finalize()
{
    // bcm2835_spi_end();
    // bcm2835_close();
    // spi_close(m_spi_fd[0]);
    // spi_close(m_spi_fd[1]);
    spiClose(m_spi_fd[0]);
    spiClose(m_spi_fd[1]);
}

// send cmd to motor and get state
void MotorSpiBridge::update()
{
    static uint8_t tx[sizeof(spine_cmd_t)] = {0};
    static uint8_t rx[sizeof(spine_cmd_t)] = {0};

    static spine_state_t temp_state;

    for(int i = 0; i < spi_count; ++i)
    {
        m_cmd[i].crc = calculate((uint8_t*)&m_cmd[i], sizeof(spine_cmd_t) - 4);

        memcpy(tx, &m_cmd[i], sizeof(spine_cmd_t));

        auto str = hex2str((uint8_t*)&m_cmd[i], sizeof(spine_cmd_t));
        std::cout << str << std::endl;

        // bcm2835_spi_chipSelect(spi_pin[i]);
        // bcm2835_spi_transfernb((char*)tx, (char*)rx, sizeof(spine_cmd_t));

        // bcm2835_gpio_write(spi_pin[i], LOW);
        // transfer(m_spi_fd[i], tx, rx, sizeof(spine_cmd_t));
        // bcm2835_gpio_write(spi_pin[i], HIGH);

        spiXfer(m_spi_fd[i], (char*)tx, (char*)rx, sizeof(spine_cmd_t));

        memcpy(&temp_state, rx, sizeof(spine_state_t));

        uint32_t crc = calculate((uint8_t*)&temp_state, sizeof(spine_state_t) - 4);
        if(crc == temp_state.crc)
        {
            std::cout << "[CRC] crc correct" << std::endl;
        }
        else
        {
            std::cout << "[CRC] crc error" << std::endl;
        }
        memcpy(&m_state[i], rx, sizeof(spine_state_t));

        str = hex2str((uint8_t*)&m_state[i], sizeof(spine_state_t));
        std::cout << str << std::endl;

        for(int j = 0; j < leg_count; ++j)
        {
            for(int k = 0; k < motor_count; ++k)
            {
                m_state[i].leg[j].motor[k].p = m_state[i].leg[j].motor[k].p * m_converter[i].leg[j].motor[k].sign + m_converter[i].leg[j].motor[k].offset;
                m_state[i].leg[j].motor[k].v = m_state[i].leg[j].motor[k].v * m_converter[i].leg[j].motor[k].sign;
                m_state[i].leg[j].motor[k].t = m_state[i].leg[j].motor[k].t * m_converter[i].leg[j].motor[k].sign;
            }
        }
    }
}

// enable motor
void MotorSpiBridge::start()
{
    for(int i = 0; i < spi_count; ++i)
    {
        for(int j = 0; j < leg_count; ++j)
        {
            m_cmd[i].leg[j].flag = 1;
        }
    }
}

// disable motor
void MotorSpiBridge::stop()
{
    for(int i = 0; i < spi_count; ++i)
    {
        for(int j = 0; j < leg_count; ++j)
        {
            m_cmd[i].leg[j].flag = 0;
        }
    }
}

void MotorSpiBridge::printInfo()
{
    std::cout << "[Robot CMD]" << std::endl;
    for(int i = 0; i < 2; ++i)
    {
        for(int j = 0; j < 2; ++j)
        {
            for(int k = 0; k < 3; ++k)
            {
                motor_cmd_t& m = m_cmd[i].leg[j].motor[k];
                std::cout << "spi: " << j << " leg: " << j << " id: " << k << " : " << m.p_des << ", " << m.v_des << ", " << m.kp << ", " << m.kd << ", " << m.t_ff << std::endl;
            }
        }
    }

    std::cout << "[Robot State]" << std::endl;
    for(int i = 0; i < 2; ++i)
    {
        for(int j = 0; j < 2; ++j)
        {
            for(int k = 0; k < 3; ++k)
            {
                motor_data_t& m = m_state[i].leg[j].motor[k];
                std::cout << "spi: " << j << " leg: " << j << " id: " << k << " : " << m.p << ", " << m.v << ", " << m.t << std::endl;
            }
        }
    }
    std::cout << std::endl;
}

// i: spi, LEFT, RIGHT, j: leg, FRONT, REAR, k:motor, ABAD, HIP, KNEE, motor cmd
void MotorSpiBridge::setJoint(int i, int j, int k, float p_des, float v_des, float kp, float kd, float t_ff)
{
    m_cmd[i].leg[j].motor[k].p_des = (p_des - m_converter[i].leg[j].motor[k].offset) / m_converter[i].leg[j].motor[k].sign;
    m_cmd[i].leg[j].motor[k].v_des = v_des / m_converter[i].leg[j].motor[k].sign;
    m_cmd[i].leg[j].motor[k].kp = kp;
    m_cmd[i].leg[j].motor[k].kd = kd;
    m_cmd[i].leg[j].motor[k].t_ff = t_ff / m_converter[i].leg[j].motor[k].sign;
}
