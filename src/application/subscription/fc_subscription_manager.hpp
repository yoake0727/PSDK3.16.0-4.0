/**
 * @file fc_subscription_manager.h
 * @brief 飞控数据订阅管理：注册主题，解析数据并通过观察者模式分发
 */

#pragma once

#include <vector>
#include <mutex>
#include <cstdint>
#include "dji_typedef.h"
#include "dji_fc_subscription.h"
#include <memory>             // 提供 std::unique_ptr
#include <chrono>             // 提供 std::chrono

/**
 * @brief 飞控数据观察者接口
 *
 * 所有需要接收飞控数据的模块均应实现此接口，并注册到 FcSubscriptionManager。
 * 回调方法在飞控数据回调线程中执行，请确保实现快速返回，避免阻塞。
 */
class IFcDataObserver
{
public:
    virtual ~IFcDataObserver() = default;

    // GPS 位置 (经纬度度, 海拔米)
    virtual void onGpsPosition(double lat, double lon, double alt) {}

    // GPS 详情 (hdop, pdop, fixState, vacc, hacc, sacc, 各卫星数等)
    virtual void onGpsInfo(dji_f32_t hdop, dji_f32_t pdop, dji_f32_t fixState,
                           dji_f32_t vacc, dji_f32_t hacc, dji_f32_t sacc,
                           uint32_t gpsSatUsed, uint32_t glonassSatUsed,
                           uint32_t totalSatUsed, uint32_t gpsCounter) {}

    // GPS 时间 (日期, 时间)
    virtual void onGpsTime(uint32_t date, uint32_t time) {}

    // RTK 状态与位置
    virtual void onRtkStatus(uint16_t status) {}
    virtual void onRtkPosition(double lat, double lon, float alt) {}

    // 速度 (NED, m/s, 总速度)
    virtual void onVelocity(double vx, double vy, double vz, double total) {}

    // 返航点 (经纬度度, 高度米)
    virtual void onHomePoint(double lat, double lon, float alt) {}

    // 融合位置 (经纬度度, 海拔米, 可见卫星数)
    virtual void onFusedPosition(double lat, double lon, float alt, uint16_t visibleSats) {}

    // 融合高度 (相对地面高度, 米)
    virtual void onFusedHeight(float height) {}

    // 姿态 (俯仰、横滚、偏航，度)
    virtual void onAttitude(double pitch, double roll, double yaw) {}

    // 飞行状态 (整型代码)
    virtual void onFlightStatus(uint8_t status) {}

    // 电池信息 (百分比, 电压mV, 温度0.1°C)
    virtual void onBatteryInfo(uint8_t percent, int32_t voltage_mv, int16_t temp_tenth_c) {}

    // 避障数据 (六个方向距离米, 各传感器健康标志, 告警标志)
    virtual void onObstacleInfo(dji_f32_t front, dji_f32_t back, dji_f32_t left, dji_f32_t right,
                                dji_f32_t up, dji_f32_t down,
                                uint8_t frontHealth, uint8_t backHealth, uint8_t leftHealth,
                                uint8_t rightHealth, uint8_t upHealth, uint8_t downHealth,
                                bool flag) {}

    // 显示模式
    virtual void onDisplayMode(uint8_t mode) {}

    // 控制设备状态 (deviceStatus, controlMode, flightStatus, vrcStatus)
    virtual void onControlAuthority(uint8_t deviceStatus, uint8_t controlMode,
                                    uint8_t flightStatus, uint8_t vrcStatus) {}
};

/**
 * @brief 飞控订阅管理器（单例风格，但允许多实例，通过静态指针访问）
 *
 * 负责订阅所有需要的 FC 主题，并将接收到的数据分发给已注册的观察者。
 * 使用全局静态指针以便在 C 回调中访问，但整个程序中只应创建一个实例。
 */
class FcSubscriptionManager
{
public:
    FcSubscriptionManager();
    ~FcSubscriptionManager();

    /**
     * @brief 初始化并订阅所有主题
     * @return true 成功，false 失败
     */
    bool start();

    /**
     * @brief 停止订阅（实际 SDK 可能不支持取消订阅，此处仅清除内部状态）
     */
    void stop();

    void addObserver(IFcDataObserver *obs);
    void removeObserver(IFcDataObserver *obs);

    // 以下方法供静态回调调用，用于分发数据
    void notifyGpsPosition(double lat, double lon, double alt);
    void notifyGpsInfo(dji_f32_t hdop, dji_f32_t pdop, dji_f32_t fixState,
                       dji_f32_t vacc, dji_f32_t hacc, dji_f32_t sacc,
                       uint32_t gpsSatUsed, uint32_t glonassSatUsed,
                       uint32_t totalSatUsed, uint32_t gpsCounter);
    void notifyGpsTime(uint32_t date, uint32_t time);
    void notifyRtkStatus(uint16_t status);
    void notifyRtkPosition(double lat, double lon, float alt);
    void notifyVelocity(double vx, double vy, double vz, double total);
    void notifyHomePoint(double lat, double lon, float alt);
    void notifyFusedPosition(double lat, double lon, float alt, uint16_t visibleSats);
    void notifyFusedHeight(float height);
    void notifyAttitude(double pitch, double roll, double yaw);
    void notifyFlightStatus(uint8_t status);
    void notifyBatteryInfo(uint8_t percent, int32_t voltage_mv, int16_t temp_tenth_c);
    void notifyObstacleInfo(dji_f32_t front, dji_f32_t back, dji_f32_t left, dji_f32_t right,
                            dji_f32_t up, dji_f32_t down,
                            uint8_t frontHealth, uint8_t backHealth, uint8_t leftHealth,
                            uint8_t rightHealth, uint8_t upHealth, uint8_t downHealth,
                            bool flag);
    void notifyDisplayMode(uint8_t mode);
    void notifyControlAuthority(uint8_t deviceStatus, uint8_t controlMode,
                                uint8_t flightStatus, uint8_t vrcStatus);

private:
    // 静态回调函数（注册给 DjiFcSubscription_SubscribeTopic）
    static T_DjiReturnCode onGpsPositionStatic(const uint8_t *data, uint16_t len, const T_DjiDataTimestamp *ts);
    static T_DjiReturnCode onGpsInfoStatic(const uint8_t *data, uint16_t len, const T_DjiDataTimestamp *ts);
    static T_DjiReturnCode onGpsTimeStatic(const uint8_t *data, uint16_t len, const T_DjiDataTimestamp *ts);
    static T_DjiReturnCode onRtkStatusStatic(const uint8_t *data, uint16_t len, const T_DjiDataTimestamp *ts);
    static T_DjiReturnCode onRtkPositionStatic(const uint8_t *data, uint16_t len, const T_DjiDataTimestamp *ts);
    static T_DjiReturnCode onVelocityStatic(const uint8_t *data, uint16_t len, const T_DjiDataTimestamp *ts);
    static T_DjiReturnCode onHomePointStatic(const uint8_t *data, uint16_t len, const T_DjiDataTimestamp *ts);
    static T_DjiReturnCode onFusedPositionStatic(const uint8_t *data, uint16_t len, const T_DjiDataTimestamp *ts);
    static T_DjiReturnCode onFusedHeightStatic(const uint8_t *data, uint16_t len, const T_DjiDataTimestamp *ts);
    static T_DjiReturnCode onQuaternionStatic(const uint8_t *data, uint16_t len, const T_DjiDataTimestamp *ts);
    static T_DjiReturnCode onFlightStatusStatic(const uint8_t *data, uint16_t len, const T_DjiDataTimestamp *ts);
    static T_DjiReturnCode onBatteryStatic(const uint8_t *data, uint16_t len, const T_DjiDataTimestamp *ts);
    static T_DjiReturnCode onAvoidDataStatic(const uint8_t *data, uint16_t len, const T_DjiDataTimestamp *ts);
    static T_DjiReturnCode onDisplayModeStatic(const uint8_t *data, uint16_t len, const T_DjiDataTimestamp *ts);
    static T_DjiReturnCode onControlAuthorityStatic(const uint8_t *data, uint16_t len, const T_DjiDataTimestamp *ts);

    // 辅助函数：订阅单个主题
    bool subscribeTopic(E_DjiFcSubscriptionTopic topic, E_DjiDataSubscriptionTopicFreq rate,
                        T_DjiReturnCode (*callback)(const uint8_t *, uint16_t, const T_DjiDataTimestamp *));

    std::vector<IFcDataObserver *> observers_;
    std::mutex mtx_;
    bool started_ = false;

    // 静态全局指针，供 C 回调使用（程序中只应有一个实例）
    static FcSubscriptionManager *instance_;
};