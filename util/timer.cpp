#include "util/timer.h"

#include <iostream>

int timer_loop(int interval, std::function<void()> task)
{
    auto next = std::chrono::steady_clock::now();
    auto prev = next - 200ms;
    while (true)
    {
        // do stuff
        auto now = std::chrono::steady_clock::now();
        std::cout << std::chrono::time_point_cast<milliseconds>(now - prev) << '\n';
        prev = now;

        task();

        // delay until time to iterate again
        next += 200ms;
        std::this_thread::sleep_until(next);
    }
}
