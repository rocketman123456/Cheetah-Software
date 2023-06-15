#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>

int timer_loop(int interval, std::function<void()> task);

class Timer
{
public:
    Timer() : m_expired(true), m_try_to_expire(false) {}

    Timer(const Timer& timer)
    {
        m_expired       = timer.m_expired.load();
        m_try_to_expire = timer.m_try_to_expire.load();
    }

    ~Timer() { stop(); }

    void start(int interval, std::function<void()> task)
    {
        // is started, do not start again
        if (m_expired == false)
            return;

        // start async timer, launch thread and wait in that thread
        m_expired = false;
        std::thread([this, interval, task]() {
            while (!m_try_to_expire)
            {
                // sleep every interval and do the task again and again until times up
                std::this_thread::sleep_for(std::chrono::milliseconds(interval));
                task();
            }

            {
                // timer be stopped, update the condition variable expired and wake main thread
                std::lock_guard<std::mutex> locker(m_mutex);
                m_expired = true;
                m_expired_cond.notify_one();
            }
        }).detach();
    }

    void startOnce(int delay, std::function<void()> task)
    {
        std::thread([delay, task]() {
            std::this_thread::sleep_for(std::chrono::milliseconds(delay));
            task();
        }).detach();
    }

    void stop()
    {
        // do not stop again
        if (m_expired)
            return;

        if (m_try_to_expire)
            return;

        // wait until timer
        m_try_to_expire = true; // change this bool value to make timer while loop stop
        {
            std::unique_lock<std::mutex> locker(m_mutex);
            m_expired_cond.wait(locker, [this] { return m_expired == true; });

            // reset the timer
            if (m_expired == true)
                m_try_to_expire = false;
        }
    }

private:
    std::atomic<bool>       m_expired;       // timer stopped status
    std::atomic<bool>       m_try_to_expire; // timer is in stop process
    std::mutex              m_mutex;
    std::condition_variable m_expired_cond;
};
