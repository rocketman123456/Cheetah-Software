#pragma once

#define MAX_STACK_SIZE 16384 // 16KB  of stack
#define TASK_PRIORITY 49     // linux priority, this is not the nice value

class HardwareBridge
{
public:
    HardwareBridge(RobotController* robot_ctrl) :
        statusTask(&taskManager, 0.5f), m_interfaceLCM(getLcmUrl(255)), // 初始化lcm实例,并链接到特殊LCM网络.可用good()函数查询是否成功
        m_visualizationLCM(getLcmUrl(255))
    {
        m_controller            = robot_ctrl;
        m_userControlParameters = robot_ctrl->getUserControlParameters();
    }

    ~HardwareBridge() { delete m_robotRunner; }

    void prefaultStack();
    void setupScheduler();
    void initError(const char* reason, bool printErrno = false);
    void initCommon();

    //void handleInterfaceLCM();
    //void handleControlParameter(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const control_parameter_request_lcmt* msg);
    //void publishVisualizationLCM();

protected:
    PeriodicTaskManager m_taskManager; // 任务管理器
    PrintTaskStatus     m_statusTask;  // 打印任务状态类,似乎取消了
    Gamepad             m_gamepad;    // 游戏手柄
    // VisualizationData m_visualizationData;                     // 可视化数据,调试用
    // CheetahVisualization m_mainCheetahVisualization;           // 在仿真环境上绘制当前机器人?
    // lcm::LCM m_interfaceLCM;                                   // lcm接口
    // lcm::LCM m_visualizationLCM;                               // 用于可视化的lcm接口
    control_parameter_respones_lcmt m_parameter_response_lcmt; // 控制参数响应lcm数据类型
    SpiData                         m_spiData;                 // spi过来的数据
    SpiCommand                      m_spiCommand;              // spi命令

    bool                   m_firstRun    = true;    // 首次运行标志
    RobotRunner*           m_robotRunner = nullptr; // 机器人运行器,比较重要
    RobotControlParameters m_robotParams;           // 机器人控制参数
    u64                    m_iterations = 0;        // 迭代器,
    std::thread            m_interfaceLcmThread;    // lcm接口线程
    volatile bool          m_interfaceLcmQuit      = false;
    RobotController*       m_controller            = nullptr;
    ControlParameters*     m_userControlParameters = nullptr;
};
