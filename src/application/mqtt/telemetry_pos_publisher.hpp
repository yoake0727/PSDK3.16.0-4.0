/**
 * @file telemetry_pos_publisher.h
 * @brief 位置/姿态高频发布器（50Hz），独立 NTP 同步
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

    // 启动 50Hz 发布线程和 NTP 同步线程
    void start();

    // 停止所有线程
    void stop();

    // 实现 IFcDataObserver 接口（只关注需要的回调）
    void onFusedPosition(double lat, double lon, float alt, uint16_t visibleSats) override;
    void onFusedHeight(float height) override;
    void onAttitude(double pitch, double roll, double yaw) override;

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

    // NTP 同步循环
    void syncLoop();

    // 执行一次 NTP 请求
    bool fetchNtpOffset(const std::string &server, double &offset);

    // 获取当前 UTC 时间戳（秒，浮点）
    double getCurrentNtpTimestamp();

    // 辅助：当前毫秒（系统时间）
    static uint64_t nowMs();

private:
    MqttBridge &mqtt_;

    PosCache cache_;
    std::mutex cache_mutex_;

    std::thread pub_thread_;
    std::atomic<bool> running_{false};

    // NTP 相关
    std::thread sync_thread_;
    std::atomic<bool> sync_running_{false};
    std::mutex offset_mutex_;
    double ntp_offset_sec_ = 0.0;
    bool ntp_initialized_ = false;
    std::string ntp_server_ = "192.168.144.130"; // 可改为从配置文件读取

    static constexpr int SYNC_INTERVAL_SEC = 30;
    static constexpr int PUBLISH_INTERVAL_MS = 20; // 50Hz = 20ms
};