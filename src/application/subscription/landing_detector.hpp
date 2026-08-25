/**
 * @file landing_detector.h
 * @brief 降落悬停检测器：监听显示模式和融合高度，检测稳定悬停并发布消息
 */

#pragma once

#include "subscription/fc_subscription_manager.hpp"
#include "mqtt/mqtt_bridge.hpp"

#include <mutex>
#include <deque>
#include <string>
#include <functional>

class LandingDetector : public IFcDataObserver
{
public:
    /**
     * @param mqtt              MQTT 桥接实例
     * @param drone_id          无人机 ID
     * @param flight_id_getter  获取当前 flightId 的回调（通常由 CommandExecutor 提供）
     */
    LandingDetector(MqttBridge &mqtt, const std::string &drone_id,
                    std::function<std::string()> flight_id_getter);
    ~LandingDetector() = default;

    // 实现 IFcDataObserver 接口
    void onDisplayMode(uint8_t mode) override;
    void onFusedHeight(float height) override;

private:
    MqttBridge &mqtt_;
    std::string drone_id_;
    std::function<std::string()> flight_id_getter_;

    // 状态变量（原 g_landing_state 内容）
    std::mutex mtx_;
    std::deque<float> height_history_; // 存储最近的高度值
    uint8_t current_mode_ = 0;         // 当前显示模式
    int stable_count_ = 0;             // 连续稳定次数
    bool hover_sent_12_ = false;       // 是否已发送 DisplayMode=12 的悬停消息
    bool hover_sent_17_ = false;       // 是否已发送 DisplayMode=17 的悬停消息
};