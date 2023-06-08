#pragma once

#include <stdint.h>

int open_socketcan(const char* name);
int close_socketcan(int s);
int transfer_socketcan(int s, int id, const char* msg, int len);
