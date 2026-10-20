/**
 * @file telemetry_pos_publisher.h
 * @brief 位置/姿态高频发布器（50Hz），统一使用系统时间
 */

#pragma once

#include <atomic>
#include <mutex>
#include <thread>
#include <string>
#include <cstdint>

#include "dji_typedef.h"
#include "mqtt_bridge.hpp"
#include "subscription/fc_subscription_manager.hpp" // IFcDataObserver
#include "local/localizer.hpp"

// 缓存结构（仅位置、姿态、高度）
struct PosCache
{
    // 融合位置
    double fused_lat = 0.0;
    double fused_lon = 0.0;
    float fused_alt = 0.0f;
    float fused_height = 0.0f; // 相对地面高度
    // 姿态（度）
    double pitch = 0.0;
    double roll = 0.0;
    double yaw = 0.0;
};

class TelemetryPosPublisher : public IFcDataObserver
{
public:
    explicit TelemetryPosPublisher(MqttBridge &mqtt);
    ~TelemetryPosPublisher();

    // 启动 50Hz 发布线程
    void start();

    // 停止所有线程
    void stop();

    // 实现 IFcDataObserver 接口（只关注需要的回调）
    void onFusedPosition(double lat, double lon, float alt, uint16_t visibleSats) override;
    void onFusedHeight(float height) override;
    void onAttitude(double pitch, double roll, double yaw) override;
    void setLocalizer(Localizer *localizer) { localizer_ = localizer; }

    // 其他回调不需要，可空实现（但必须提供）
    void onGpsPosition(double, double, double) override {}
    void onGpsInfo(dji_f32_t, dji_f32_t, dji_f32_t, dji_f32_t, dji_f32_t, dji_f32_t,
                   uint32_t, uint32_t, uint32_t, uint32_t) override {}
    void onGpsTime(uint32_t, uint32_t) override {}
    void onRtkStatus(uint16_t) override {}
    void onRtkPosition(double, double, float) override {}
    void onVelocity(double, double, double, double) override {}
    void onHomePoint(double, double, float) override {}
    void onFlightStatus(uint8_t) override {}
    void onBatteryInfo(uint8_t, int32_t, int16_t) override {}
    void onObstacleInfo(dji_f32_t, dji_f32_t, dji_f32_t, dji_f32_t,
                        dji_f32_t, dji_f32_t, uint8_t, uint8_t, uint8_t,
                        uint8_t, uint8_t, uint8_t, bool) override {}
    void onDisplayMode(uint8_t) override {}
    void onControlAuthority(uint8_t, uint8_t, uint8_t, uint8_t) override {}

private:
    // 50Hz 循环
    void runLoop();

    // 发布一次快照（构造 JSON 并发送）
    void publishPos();

    // 辅助：当前毫秒（妙算3系统时间，统一时间基准）
    static uint64_t nowMs();

private:
    MqttBridge &mqtt_;

    PosCache cache_;
    std::mutex cache_mutex_;

    std::thread pub_thread_;
    std::atomic<bool> running_{false};

    Localizer *localizer_ = nullptr;

    static constexpr int PUBLISH_INTERVAL_MS = 20; // 50Hz = 20ms
};
