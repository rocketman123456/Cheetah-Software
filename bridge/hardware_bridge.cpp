#include "bridge/hardware_bridge.h"
#include "hardware/motor_spi.h"

#include "rt/rt_util.h"
#include "util/crc.h"

#include <iostream>
#include <chrono>
#include <thread>
#define _USE_MATH_DEFINES
#include <math.h>

const char* name[] = {"/dev/spidev0.0", "/dev/spidev0.1"};

const uint32_t spi_count = 2;
const uint32_t leg_count = 2;
const uint32_t motor_count = 3;

// init communication
void HardwareBridge::initialize()
{
    //spi_open(m_spi_fd[0], name[0]);
    //spi_open(m_spi_fd[1], name[1]);

    float angle = M_PI / 2.0 - 26.0 * M_PI / 180.0;

    m_converter[LEFT].leg[FRONT].motor[ABAD].sign = -1;
    m_converter[LEFT].leg[FRONT].motor[ABAD].offset = 0;
    m_converter[LEFT].leg[FRONT].motor[HIP].sign = -1;
    m_converter[LEFT].leg[FRONT].motor[HIP].offset = - M_PI / 2.0;
    m_converter[LEFT].leg[FRONT].motor[KNEE].sign = 1;
    m_converter[LEFT].leg[FRONT].motor[KNEE].offset = -angle;

    m_converter[LEFT].leg[REAR].motor[ABAD].sign = 1;
    m_converter[LEFT].leg[REAR].motor[ABAD].offset = 0;
    m_converter[LEFT].leg[REAR].motor[HIP].sign = -1;
    m_converter[LEFT].leg[REAR].motor[HIP].offset = M_PI / 2.0;
    m_converter[LEFT].leg[REAR].motor[KNEE].sign = 1;
    m_converter[LEFT].leg[REAR].motor[KNEE].offset = angle;

    m_converter[RIGHT].leg[FRONT].motor[ABAD].sign = -1;
    m_converter[RIGHT].leg[FRONT].motor[ABAD].offset = 0;
    m_converter[RIGHT].leg[FRONT].motor[HIP].sign = 1;
    m_converter[RIGHT].leg[FRONT].motor[HIP].offset = - M_PI / 2.0;
    m_converter[RIGHT].leg[FRONT].motor[KNEE].sign = -1;
    m_converter[RIGHT].leg[FRONT].motor[KNEE].offset = -angle;

    m_converter[RIGHT].leg[REAR].motor[ABAD].sign = 1;
    m_converter[RIGHT].leg[REAR].motor[ABAD].offset = 0;
    m_converter[RIGHT].leg[REAR].motor[HIP].sign = 1;
    m_converter[RIGHT].leg[REAR].motor[HIP].offset = M_PI / 2.0;
    m_converter[RIGHT].leg[REAR].motor[KNEE].sign = -1;
    m_converter[RIGHT].leg[REAR].motor[KNEE].offset = angle;

    float default_kp = 0.4;

    // init a very soft behavior
    this->setJoint(LEFT, FRONT, ABAD, 0, 0, default_kp, 0, 0);
    this->setJoint(LEFT, FRONT, HIP, -M_PI / 2.0, 0, default_kp, 0, 0);
    this->setJoint(LEFT, FRONT, KNEE, 0, 0, default_kp, 0, 0);

    this->setJoint(LEFT, REAR, ABAD, 0, 0, default_kp, 0, 0);
    this->setJoint(LEFT, REAR, HIP, -M_PI / 2.0, 0, default_kp, 0, 0);
    this->setJoint(LEFT, REAR, KNEE, 0, 0, default_kp, 0, 0);

    this->setJoint(RIGHT, FRONT, ABAD, 0, 0, default_kp, 0, 0);
    this->setJoint(RIGHT, FRONT, HIP, -M_PI / 2.0, 0, default_kp, 0, 0);
    this->setJoint(RIGHT, FRONT, KNEE, 0, 0, default_kp, 0, 0);

    this->setJoint(RIGHT, REAR, ABAD, 0, 0, default_kp, 0, 0);
    this->setJoint(RIGHT, REAR, HIP, -M_PI / 2.0, 0, default_kp, 0, 0);
    this->setJoint(RIGHT, REAR, KNEE, 0, 0, default_kp, 0, 0);
}

void HardwareBridge::finalize()
{
    spi_close(m_spi_fd[0]);
    spi_close(m_spi_fd[1]);
}

// send cmd to motor and get state
void HardwareBridge::update()
{
    static uint8_t tx[sizeof(spine_cmd_t)] = {0};
    static uint8_t rx[sizeof(spine_cmd_t)] = {0};

    static spine_state_t temp_state;

    for(int i = 0; i < spi_count; ++i)
    {
        m_cmd[i].crc = calculate((uint8_t*)&m_cmd[i], sizeof(spine_cmd_t) - 4);

        memcpy(tx, &m_cmd[i], sizeof(spine_cmd_t));

        spi_open(m_spi_fd[i], name[i]);
        int rv = transfer(m_spi_fd[i], tx, rx, sizeof(spine_cmd_t));
        (void)rv;
        spi_close(m_spi_fd[i]);

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

        for(int j = 0; j < leg_count; ++j)
        {
            for(int k = 0; k < motor_count; ++j)
            {
                m_state[i].leg[j].motor[k].p = m_state[i].leg[j].motor[k].p * m_converter[i].leg[j].motor[k].sign + m_converter[i].leg[j].motor[k].offset;
                m_state[i].leg[j].motor[k].v = m_state[i].leg[j].motor[k].v * m_converter[i].leg[j].motor[k].sign;
                m_state[i].leg[j].motor[k].t = m_state[i].leg[j].motor[k].t * m_converter[i].leg[j].motor[k].sign;
            }
        }
    }
}

// enable motor
void HardwareBridge::start()
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
void HardwareBridge::stop()
{
    for(int i = 0; i < spi_count; ++i)
    {
        for(int j = 0; j < leg_count; ++j)
        {
            m_cmd[i].leg[j].flag = 0;
        }
    }
}

void HardwareBridge::printInfo()
{
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
void HardwareBridge::setJoint(int i, int j, int k, float p_des, float v_des, float kp, float kd, float t_ff)
{
    m_cmd[i].leg[j].motor[k].p_des = (p_des - m_converter[i].leg[j].motor[k].offset) / m_converter[i].leg[j].motor[k].sign;
    m_cmd[i].leg[j].motor[k].v_des = v_des / m_converter[i].leg[j].motor[k].sign;
    m_cmd[i].leg[j].motor[k].kp = kp;
    m_cmd[i].leg[j].motor[k].kd = kd;
    m_cmd[i].leg[j].motor[k].t_ff = t_ff / m_converter[i].leg[j].motor[k].sign;
}
