#include "util/timer.h"

#include <iostream>

int timer_loop(int interval, std::function<void()> task)
{
    using namespace std::chrono_literals;
    using namespace std::chrono;
    auto duration = 200ms;
    steady_clock::time_point next = steady_clock::now();
    steady_clock::time_point prev = next - duration;
    while (true)
    {
        // do stuff
        steady_clock::time_point now = steady_clock::now();
        std::cout << duration_cast<microseconds>(now - prev).count() << '\n';
        prev = now;

        task();

        // delay until time to iterate again
        next += duration;
        std::this_thread::sleep_until(next);
    }
}
