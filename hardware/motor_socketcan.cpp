#include "hardware/motor_socketcan.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

int open_socketcan(const char* name)
{
    struct sockaddr_can addr;
    struct ifreq ifr;

    printf("Start CAN Socket %s\n", name);

    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
    {
        perror("[Error] Socket Open Error");
        return 1;
    }

    strcpy(ifr.ifr_name, name);
    ioctl(s, SIOCGIFINDEX, &ifr);

    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        perror("[Error] Socket Bind Error");
        return 1;
    }
    return 0;
}

int close_socketcan(int s)
{
    if (close(s) < 0)
    {
        perror("[Error] Socket Close Error");
        return 1;
    }
    return 0;
}

int transfer_socketcan(int s, int id, const char* msg, int len)
{
    struct can_frame frame;
    frame.can_id = id;
    frame.can_dlc = len;
    sprintf(frame.data, msg);

    if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
    {
        perror("[Error] Socket Write Error");
        return 1;
    }
    return 0;
}
