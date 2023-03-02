/*!
 * @file rt_serial.h
 * @brief Serial port
 */
#pragma once

int set_interface_attribs_custom_baud(int fd, int speed, int parity, int port);
void init_serial_for_sbus(int fd, int baud);
