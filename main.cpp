#include "rt_spi.h"
#include "motor_control.h"
#include "crc.h"

#include <cmath>
#include <iostream>
#include <unistd.h>

int spi_1_fd = -1;
int spi_2_fd = -1;

const char* name1 = "/dev/spidev0.0";
const char* name2 = "/dev/spidev0.1";

spine_cmd_t   cmd[2];
spine_state_t state[2];

using namespace std;

int main()
{
    cout << "spine_cmd_t: " << sizeof(spine_cmd_t) << endl;
    cout << "spine_state_t: " << sizeof(spine_state_t) << endl;

    create_lookup_table();

    spi_open(spi_1_fd, name1);

    uint8_t tx[sizeof(spine_cmd_t)] = {0};
    uint8_t rx[sizeof(spine_cmd_t)] = {0};

    uint16_t* tx_buf = (uint16_t *)tx;
    uint16_t* rx_buf = (uint16_t *)rx;

    uint16_t *cmd_d = (uint16_t *)&cmd[0];
    uint16_t *data_d = (uint16_t *)&state[0];

    cmd[0].cmd[0].flag = 1;
    cmd[0].cmd[0].cmd[0].kp = 5;

    cmd[0].crc = calculate((uint8_t*)&cmd[0], sizeof(spine_cmd_t) - 4);

    auto str = hex2str((uint8_t*)&cmd[0], sizeof(spine_cmd_t));
    cout << str << endl << endl;

    memcpy(tx, &cmd[0], sizeof(spine_cmd_t));
    //for (int i = 0; i < sizeof(spine_cmd_t) / 2; i++)
    //    tx_buf[i] = (cmd_d[i] >> 8) + ((cmd_d[i] & 0xff) << 8);

    str = hex2str(tx, sizeof(spine_cmd_t));
    cout << str << endl << endl;

    int rv = transfer(spi_1_fd, tx, rx, sizeof(spine_cmd_t));
    (void)rv;

    memcpy(&state[0], rx, sizeof(spine_state_t));
    //for (int i = 0; i < sizeof(spine_state_t) / 2; i++)
    //    data_d[i] = (rx_buf[i] >> 8) + ((rx_buf[i] & 0xff) << 8);

    uint32_t crc = calculate((uint8_t*)&state[0], sizeof(spine_state_t) - 4);

    if(crc == state[0].crc)
    {
        cout << "[CRC] crc correct" << endl;
    }
    else
    {
        cout << "[CRC] crc error" << endl;
    }

    for(int i = 0; i < 2; ++i)
    {
        leg_state_t& s = state[0].state[i];
        for(int j = 0; j < 3; ++j)
        {
            motor_data_t& m = s.state[j];
            cout << "leg: " << i << " id: " << j << " : " << m.p << ", " << m.v << ", " << m.t << endl;
        }
    }

    spi_close(spi_1_fd);

    return 0;
}
