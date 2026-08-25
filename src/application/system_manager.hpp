#pragma once

#include "dji_typedef.h"   // 提供 T_DjiReturnCode
#include <memory>          // std::unique_ptr
#include <chrono>          // std::chrono::steady_clock
#include <atomic>
#include <string>

class MqttBridge;
class TelemetryPublisher;
class TelemetryPosPublisher;
class CommandExecutor;
class FcSubscriptionManager;
class LandingDetector;
class H30tStreamController;

class SystemManager
{
public:
    SystemManager();
    ~SystemManager();

    /**
     * @brief 初始化系统管理器
     * @return true 成功，false 失败
     */
    bool init();
    /**
     * @brief 关闭系统管理器，释放所有资源
     */
    void shutdown();
    /**
     * @brief 获取无人机 ID
     */
    const std::string& droneId() const { return drone_id_; }
    /**
     * @brief 获取 H30T 流控制器（供 CommandExecutor 使用）
     */
    H30tStreamController* getH30tStreamController() const {
        return h30t_stream_.get();
    }

private:
    void loadMqttConfig();   // 如果不需要可以从头文件中移除声明
    static T_DjiReturnCode mqttLogCallback(const uint8_t *data, uint16_t dataLen);

    /**
     * @brief 启动 H30T 视频流并向外部 RTSP 服务器推流
     * @return true 成功，false 失败
     */
    bool startH30tStream();

    /**
     * @brief 启动 MQTT 依赖的服务（在 MQTT 连接成功后调用）
     * @return true 成功，false 失败
     */
    bool startMqttDependencies();

    /**
     * @brief 停止 H30T 视频流
     */
    void stopH30tStream();

    /**
     * @brief 停止 MQTT 依赖的服务
     */
    void stopMqttDependencies();


private:
    std::unique_ptr<MqttBridge> mqtt_;
    std::unique_ptr<TelemetryPublisher> telemetry_;
    std::unique_ptr<TelemetryPosPublisher> telemetry_pos_;
    std::unique_ptr<CommandExecutor> cmd_exec_;
    std::unique_ptr<FcSubscriptionManager> fc_mgr_;
    std::unique_ptr<LandingDetector> landing_detector_;

    // === H30T 视频流控制 ===
    std::unique_ptr<H30tStreamController> h30t_stream_;
    int h30t_mount_ = 1;  // 默认挂载点 1

    std::atomic<bool> running_{false};
    std::string drone_id_;
    std::chrono::steady_clock::time_point start_time_;
};
