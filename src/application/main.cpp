/**
 * @file main.cpp
 * @brief 程序入口：启动 DJI PSDK 原版应用，然后加载自定义模块（MQTT，遥测，命令，降落检测等）
 */

#include "application.hpp"       // 原版 Application 类
#include "system_manager.hpp"      // 自定义系统管理器
#include "../common/osal/osal.h" // Osal_TaskSleepMs
#include "dji_logger.h"

#include <csignal>
#include <iostream>
#include <cstdlib>

extern "C" void SetSystemManagerForExit(SystemManager *mgr);


int main(int argc, char **argv)
{
    system("cd ~/mediamtx && nohup ./mediamtx > /dev/null 2>&1 &");
    system("mkdir -p data/logs");
    system("/system/bin/netctl.sh setup_static_ip --type eth --iface eth0 --ip 192.168.144.45 --gw 192.168.144.44 --mask 24");
    // 1. 启动 DJI PSDK 原版应用
    signal(SIGTERM, [](int signalNum) -> void { exit(0); });
    Application application(argc, argv);
          USER_LOG_INFO("KAISHI");
    char inputChar;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    T_DjiReturnCode returnCode;
    E_DjiMountPosition mountPosition = DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1;

    // 等待 PSDK 内部线程稳定（可选，根据实际情况调整）
    // 原版 Application 构造函数中有一个 Osal_TaskSleepMs(3000)，此处再等待 2 秒确保核心就绪
    Osal_TaskSleepMs(2000);
    USER_LOG_INFO("[NODE][main] PSDK stabilization wait completed");

    // 2. 创建并初始化自定义系统管理器（MQTT，遥测，命令处理，FC 订阅，降落检测等）
    SystemManager sys_mgr;
    USER_LOG_INFO("[NODE][main] SystemManager constructed");
    // g_system_manager = &sys_mgr;

    SetSystemManagerForExit(&sys_mgr);
    USER_LOG_INFO("[NODE][main] SystemManager registered for exit handling");

    if (!sys_mgr.init())
    {
        USER_LOG_ERROR("[NODE][main] SystemManager initialization failed");
        return 1;
    }

    // 3. 主线程等待（信号处理函数会调用 sys_mgr.shutdown() 并 exit）    
    USER_LOG_INFO("[NODE][main] SystemManager started; waiting for shutdown signal");
    while (true)
    {
        Osal_TaskSleepMs(1000);
    }

    // 永远不会执行到这里，但保留 return 以避免编译器警告
    return 0;
}
