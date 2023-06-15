#pragma once

// 时间戳  微秒级
long long get_system_time();
// 时间戳  秒级
double get_time_second();
// 等待函数，微秒级，从startTime开始等待waitTime微秒
void absolute_wait(long long startTime, long long waitTime)