#pragma once

#define MAX_STACK_SIZE 16384 // 16KB  of stack
#define TASK_PRIORITY 49     // linux priority, this is not the nice value

class HardwareBridge
{
public:
    HardwareBridge(RobotController* robot_ctrl) :
        statusTask(&taskManager, 0.5f), _interfaceLCM(getLcmUrl(255)), // 初始化lcm实例,并链接到特殊LCM网络.可用good()函数查询是否成功
        _visualizationLCM(getLcmUrl(255))
    {
        _controller            = robot_ctrl;
        _userControlParameters = robot_ctrl->getUserControlParameters();
    }

    void prefaultStack();
    void setupScheduler();
    void initError(const char* reason, bool printErrno = false);
    void initCommon();
    ~HardwareBridge() { delete _robotRunner; }

    void handleInterfaceLCM();
    void handleControlParameter(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const control_parameter_request_lcmt* msg);

    void publishVisualizationLCM();

protected:
    PeriodicTaskManager taskManager; // 任务管理器
    PrintTaskStatus     statusTask;  // 打印任务状态类,似乎取消了
    Gamepad             _gamepad;    // 游戏手柄
    // VisualizationData _visualizationData;                     // 可视化数据,调试用
    // CheetahVisualization _mainCheetahVisualization;           // 在仿真环境上绘制当前机器人?
    // lcm::LCM _interfaceLCM;                                   // lcm接口
    // lcm::LCM _visualizationLCM;                               // 用于可视化的lcm接口
    control_parameter_respones_lcmt _parameter_response_lcmt; // 控制参数响应lcm数据类型
    SpiData                         _spiData;                 // spi过来的数据
    SpiCommand                      _spiCommand;              // spi命令

    bool                   _firstRun    = true;    // 首次运行标志
    RobotRunner*           _robotRunner = nullptr; // 机器人运行器,比较重要
    RobotControlParameters _robotParams;           // 机器人控制参数
    u64                    _iterations = 0;        // 迭代器,
    std::thread            _interfaceLcmThread;    // lcm接口线程
    volatile bool          _interfaceLcmQuit      = false;
    RobotController*       _controller            = nullptr;
    ControlParameters*     _userControlParameters = nullptr;
};
