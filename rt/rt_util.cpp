#include "rt/rt_util.h"

#include <sys/mman.h>
#include <unistd.h>

#define MAX_STACK_SIZE 16384
#define TASK_PRIORITY 49

/*!
 * Writes to a 16 KB buffer on the stack. If we are using 4K pages for our
 * stack, this will make sure that we won't have a page fault when the stack
 * grows.  Also mlock's all pages associated with the current process, which
 * prevents the cheetah software from being swapped out.  If we do run out of
 * memory, the robot program will be killed by the OOM process killer (and
 * leaves a log) instead of just becoming unresponsive.
 * 写入到堆栈上的一个16 KB的缓冲区。如果我们为堆栈使用4K页面，这将确保在堆栈增长时不会出现页面错误。
 * 还有mlock与当前进程关联的所有页面，这可以防止cheetah软件被换出。
 * 如果内存耗尽，机器人程序将被OOM进程杀手杀死(并留下日志)，而不是变得没有响应。
 * 注意,需要root权限
 */
void prefaultStack()
{
    printf("[Init] Prefault stack...\n");
    volatile char stack[MAX_STACK_SIZE]; // MAX_STACK_SIZE=16384 16kb
    memset(const_cast<char*>(stack), 0, MAX_STACK_SIZE);
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1)
    {
        printf("mlockall failed.  This is likely because you didn't run robot as root.\n");
        exit(-1);
    }
}

/*!
 * Configures the scheduler for real time priority
 * 如果pid为零，将会为调用进程设置调度策略和调度参数。
 * 如果进程pid含多个进程或轻量进程(即该进程是多进程的)，此函数将影响进程中各个子进程。
 */
void setupScheduler()
{
    printf("[Init] Setup RT Scheduler...\n");
    pid_t              pid = getpid();
    struct sched_param params;
    params.sched_priority = sched_get_priority_max(SCHED_FIFO); // TASK_PRIORITY  = 49
    if (sched_setscheduler(pid, SCHED_FIFO, &params) == -1)
    {
        printf("[ERROR] sched_setscheduler failed.\n");
        exit(-1);
    }
}
