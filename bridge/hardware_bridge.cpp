#include "bridge/hardware_bridge.h"
#include "hardware/motor_spi.h"

#include "rt/rt_util.h"
#include "util/crc.h"

#include <iostream>

const char* name[] = {"/dev/spidev0.0", "/dev/spidev0.1"};

const uint32_t spi_count = 2;
const uint32_t leg_count = 2;
const uint32_t motor_count = 3;

// init communication
void HardwareBridge::initialize()
{
    //prefaultStack();
    //setupScheduler();

    //create_lookup_table();

    spi_open(m_spi_fd[0], name[0]);
    spi_open(m_spi_fd[1], name[1]);
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

    spine_state_t temp_state;

    for(int i = 0; i < spi_count; ++i)
    {
        m_cmd[i].crc = calculate((uint8_t*)&m_cmd[i], sizeof(spine_cmd_t) - 4);

        memcpy(tx, &m_cmd[i], sizeof(spine_cmd_t));

        int rv = transfer(m_spi_fd[i], tx, rx, sizeof(spine_cmd_t));
        (void)rv;

        memcpy(&temp_state[i], rx, sizeof(spine_state_t));

        uint32_t crc = calculate((uint8_t*)&temp_state, sizeof(spine_state_t) - 4);
        if(crc == state[i].crc)
        {
            memcpy(&m_state[i], rx, sizeof(spine_state_t));
            std::cout << "[CRC] crc correct" << std::endl;
        }
        else
        {
            std::cout << "[CRC] crc error" << std::endl;
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
                std::cout << "spi: " << j << "leg: " << j << " id: " << k << " : " << m.p << ", " << m.v << ", " << m.t << std::endl;
            }
        }
    }
    std::cout << std::endl;
}

// i: spi, LEFT, RIGHT, j: leg, FRONT, REAR, k:motor, ABAD, HIP, KNEE, motor cmd
void HardwareBridge::setJoint(int i, int j, int k, float p_des, float v_des, float kp, float kd, float t_ff)
{
    m_cmd[i].leg[j].motor[k].p_des = p_des;
    m_cmd[i].leg[j].motor[k].v_des = v_des;
    m_cmd[i].leg[j].motor[k].kp = kp;
    m_cmd[i].leg[j].motor[k].kd = kd;
    m_cmd[i].leg[j].motor[k].t_ff = t_ff;
}
