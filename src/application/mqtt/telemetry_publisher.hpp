/**
 * @file telemetry_publisher.h
 * @brief 遥测数据发布器：接收 FC 数据缓存，定时 5Hz 发布 JSON 到 MQTT
 */

#pragma once

#include <atomic>
#include <mutex>
#include <thread>
#include <string>
#include <cstdint>
#include "dji_typedef.h"
#include "mqtt_bridge.hpp"
#include "subscription/fc_subscription_manager.hpp" // 引入 IFcDataObserver

// 遥测数据缓存结构（线程安全，由 TelemetryPublisher 内部锁保护）
struct TelemetryCache
{
    // 时间戳 (ms since epoch)
    uint64_t ts_utc = 0;

    // GPS 时间 (日期, 时间)
    uint32_t gps_date = 0;
    uint32_t gps_time = 0;

    // GPS 信息
    dji_f32_t hdop = 0.0f, pdop = 0.0f, fixState = 0.0f;
    dji_f32_t vacc = 0.0f, hacc = 0.0f, sacc = 0.0f;
    uint32_t gps_sat_used = 0, glonass_sat_used = 0, total_sat_used = 0, gps_counter = 0;

    // GPS 位置 (度, 度, 米)
    double gps_lat = 0.0, gps_lon = 0.0, gps_alt = 0.0;

    // RTK
    uint16_t rtk_status = 0;
    double rtk_lat = 0.0, rtk_lon = 0.0;
    float rtk_alt = 0.0f;

    // 速度 (m/s)
    double vel_x = 0.0, vel_y = 0.0, vel_z = 0.0, vel_total = 0.0;

    // 返航点
    double home_lat = 0.0, home_lon = 0.0;
    float home_alt = 0.0f;

    // 融合位置
    double fused_lat = 0.0, fused_lon = 0.0;
    float fused_alt = 0.0f;
    uint16_t visible_sats = 0;

    // 融合高度 (相对地面)
    float fused_height = 0.0f;

    // 姿态 (度)
    double pitch = 0.0, roll = 0.0, yaw = 0.0;

    // 飞行状态
    uint8_t flight_status = 0;

    // 电池 (百分比, 电压mV, 温度0.1°C)
    uint8_t battery_percent = 0;
    int32_t battery_voltage_mv = 0;
    int16_t battery_temp_tenth_c = 0;

    // 避障 (距离米, 健康标志, 告警)
    dji_f32_t obs_front = 0.0f, obs_back = 0.0f, obs_left = 0.0f, obs_right = 0.0f, obs_up = 0.0f, obs_down = 0.0f;
    uint8_t obs_front_health = 0, obs_back_health = 0, obs_left_health = 0, obs_right_health = 0, obs_up_health = 0, obs_down_health = 0;
    bool obs_flag = false;

    // 显示模式
    uint8_t display_mode = 0;

    // 控制权设备状态
    uint8_t control_device_status = 0;
    uint8_t control_mode = 0;
    uint8_t control_flight_status = 0;
    uint8_t vrc_status = 0;

    // 任务 ID (由外部 set 方法写入)
    std::string waypoint_flightid;

    // 是否有摇杆控制权
    bool has_joystick_control = false;
};

class TelemetryPublisher : public IFcDataObserver
{
public:
    explicit TelemetryPublisher(MqttBridge &mqtt);
    ~TelemetryPublisher();

    // 启动 5Hz 发布线程
    void start_5hz();

    // 停止发布线程
    void stop();

    // 外部设置任务 ID (线程安全，供 CommandExecutor 调用)
    void setWaypointFlightId(const std::string &id);

    // 设置摇杆控制权状态 (供 CommandExecutor 调用)
    void setHasJoystickControl(bool has);

    // ---------- 实现 IFcDataObserver 接口 ----------
    void onGpsPosition(double lat, double lon, double alt) override;
    void onGpsInfo(dji_f32_t hdop, dji_f32_t pdop, dji_f32_t fixState,
                   dji_f32_t vacc, dji_f32_t hacc, dji_f32_t sacc,
                   uint32_t gpsSatUsed, uint32_t glonassSatUsed,
                   uint32_t totalSatUsed, uint32_t gpsCounter) override;
    void onGpsTime(uint32_t date, uint32_t time) override;
    void onRtkStatus(uint16_t status) override;
    void onRtkPosition(double lat, double lon, float alt) override;
    void onVelocity(double vx, double vy, double vz, double total) override;
    void onHomePoint(double lat, double lon, float alt) override;
    void onFusedPosition(double lat, double lon, float alt, uint16_t visibleSats) override;
    void onFusedHeight(float height) override;
    void onAttitude(double pitch, double roll, double yaw) override;
    void onFlightStatus(uint8_t status) override;
    void onBatteryInfo(uint8_t percent, int32_t voltage_mv, int16_t temp_tenth_c) override;
    void onObstacleInfo(dji_f32_t front, dji_f32_t back, dji_f32_t left, dji_f32_t right,
                        dji_f32_t up, dji_f32_t down,
                        uint8_t frontHealth, uint8_t backHealth, uint8_t leftHealth,
                        uint8_t rightHealth, uint8_t upHealth, uint8_t downHealth,
                        bool flag) override;
    void onDisplayMode(uint8_t mode) override;
    void onControlAuthority(uint8_t deviceStatus, uint8_t controlMode,
                            uint8_t flightStatus, uint8_t vrcStatus) override;

private:
    // 5Hz 循环函数
    void runLoop();

    // 从缓存快照并发布 JSON
    void publishSnapshot();

    // 辅助：获取当前 UTC 毫秒时间戳
    static uint64_t nowMs();

private:
    MqttBridge &mqtt_;
    TelemetryCache cache_;
    std::mutex cache_mutex_;

    std::thread publish_thread_;
    std::atomic<bool> running_{false};
};