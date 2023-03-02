#pragma once

#include <cstdint>

void rt_ethercat_init();
void rt_ethercat_run();

struct TiBoardData;
struct TiBoardCommand;

void rt_ethercat_get_data(TiBoardData* data);
void rt_ethercat_set_command(TiBoardCommand* command);
