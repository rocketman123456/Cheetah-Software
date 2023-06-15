#include <iostream>
#include <sys/time.h>
#include <unistd.h>

// 时间戳  微秒级， 需要#include <sys/time.h>
long long get_system_time()
{
    struct timeval t;
    gettimeofday(&t, NULL);
    return 1000000 * t.tv_sec + t.tv_usec;
}

// 时间戳  秒级， 需要getSystemTime()
double get_time_second()
{
    double time = get_system_time() * 0.000001;
    return time;
}

// 等待函数，微秒级，从startTime开始等待waitTime微秒
void absolute_wait(long long startTime, long long waitTime)
{
    if (get_system_time() - startTime > waitTime)
    {
        std::cout << "[WARNING] The waitTime=" << waitTime << " of function absoluteWait is not enough!" << std::endl
                  << "The program has already cost " << get_system_time() - startTime << "us." << std::endl;
    }
    while (get_system_time() - startTime < waitTime)
    {
        usleep(50);
    }
}