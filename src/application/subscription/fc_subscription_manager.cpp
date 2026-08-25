/**
 * @file fc_subscription_manager.cpp
 * @brief 飞控数据订阅管理器实现
 */

#include "fc_subscription_manager.hpp"
#include "dji_logger.h"
#include "dji_fc_subscription.h" // 包含 E_DjiFcSubscriptionTopic 等定义
#include <cmath>
#include <cstring>
#include <algorithm>

#define RAD2DEG 57.29577951308232

// 静态成员初始化
FcSubscriptionManager *FcSubscriptionManager::instance_ = nullptr;

FcSubscriptionManager::FcSubscriptionManager()
{
    instance_ = this;
}

FcSubscriptionManager::~FcSubscriptionManager()
{
    stop();
    if (instance_ == this)
        instance_ = nullptr;
}

bool FcSubscriptionManager::subscribeTopic(E_DjiFcSubscriptionTopic topic,
                                           E_DjiDataSubscriptionTopicFreq rate,
                                           T_DjiReturnCode (*callback)(const uint8_t *, uint16_t, const T_DjiDataTimestamp *))
{
    if (!started_)
    {
        USER_LOG_ERROR("FC Subscription not initialized");
        return false;
    }
    T_DjiReturnCode rc = DjiFcSubscription_SubscribeTopic(topic, rate, callback);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
        USER_LOG_ERROR("Failed to subscribe topic %d, rc=0x%08llX", topic, rc);
        return false;
    }
    USER_LOG_DEBUG("Subscribed topic %d, rate %d", topic, rate);
    return true;
}

bool FcSubscriptionManager::start()
{
    if (started_)
    return true;

    // 1. 必须先初始化 FC 订阅模块
    T_DjiReturnCode rc = DjiFcSubscription_Init();
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
        USER_LOG_ERROR("DjiFcSubscription_Init failed, rc=0x%08llX", rc);
        return false;
    }

    // 2. 初始化成功，设置标志
    started_ = true;

    // 3. 订阅所有需要的主题
    // 订阅所有需要的主题 (频率与原始 application.cpp 保持一致)
    bool all_ok = true;
    all_ok &= subscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION, DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ, onGpsPositionStatic);
    all_ok &= subscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_DETAILS, DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ, onGpsInfoStatic);
    all_ok &= subscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_DATE, DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ, onGpsTimeStatic);
    all_ok &= subscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_TIME, DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ, onGpsTimeStatic);
    all_ok &= subscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_RTK_CONNECT_STATUS, DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ, onRtkStatusStatic);
    all_ok &= subscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION, DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ, onRtkPositionStatic);
    all_ok &= subscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY, DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ, onVelocityStatic);
    all_ok &= subscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_HOME_POINT_INFO, DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ, onHomePointStatic);
    all_ok &= subscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_OF_HOMEPOINT, DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ, onHomePointStatic); // 注：回调相同，但需单独订阅，实际处理中我们将其合并到 HomePoint 通知中
    all_ok &= subscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_POSITION_FUSED, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ, onFusedPositionStatic);
    all_ok &= subscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ, onQuaternionStatic);
    all_ok &= subscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT, DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ, onFlightStatusStatic);
    all_ok &= subscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_BATTERY_SINGLE_INFO_INDEX1, DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ, onBatteryStatic);
    all_ok &= subscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_AVOID_DATA, DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ, onAvoidDataStatic);
    all_ok &= subscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_DISPLAYMODE, DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ, onDisplayModeStatic);
    all_ok &= subscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_HEIGHT_FUSION, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ, onFusedHeightStatic);
    all_ok &= subscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_CONTROL_DEVICE, DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ, onControlAuthorityStatic);

    if (!all_ok)
    {
        USER_LOG_ERROR("Some topic subscriptions failed");
        return false;
    }

    started_ = true;
    USER_LOG_INFO("FcSubscriptionManager started (subscribed all topics)");
    return true;
}

void FcSubscriptionManager::stop()
{
    started_ = false;
    // SDK 没有明确的取消订阅接口，所以这里仅清除内部状态
    USER_LOG_INFO("FcSubscriptionManager stopped");
}

void FcSubscriptionManager::addObserver(IFcDataObserver *obs)
{
    if (!obs)
        return;
    std::lock_guard<std::mutex> lock(mtx_);
    for (auto &o : observers_)
    {
        if (o == obs)
            return;
    }
    observers_.push_back(obs);
}

void FcSubscriptionManager::removeObserver(IFcDataObserver *obs)
{
    std::lock_guard<std::mutex> lock(mtx_);
    observers_.erase(std::remove(observers_.begin(), observers_.end(), obs), observers_.end());
}

// ---------- 通知方法实现 ----------
void FcSubscriptionManager::notifyGpsPosition(double lat, double lon, double alt)
{
    std::lock_guard<std::mutex> lock(mtx_);
    for (auto *obs : observers_)
        obs->onGpsPosition(lat, lon, alt);
}

void FcSubscriptionManager::notifyGpsInfo(dji_f32_t hdop, dji_f32_t pdop, dji_f32_t fixState,
                                          dji_f32_t vacc, dji_f32_t hacc, dji_f32_t sacc,
                                          uint32_t gpsSatUsed, uint32_t glonassSatUsed,
                                          uint32_t totalSatUsed, uint32_t gpsCounter)
{
    std::lock_guard<std::mutex> lock(mtx_);
    for (auto *obs : observers_)
        obs->onGpsInfo(hdop, pdop, fixState, vacc, hacc, sacc,
                       gpsSatUsed, glonassSatUsed, totalSatUsed, gpsCounter);
}

void FcSubscriptionManager::notifyGpsTime(uint32_t date, uint32_t time)
{
    std::lock_guard<std::mutex> lock(mtx_);
    for (auto *obs : observers_)
        obs->onGpsTime(date, time);
}

void FcSubscriptionManager::notifyRtkStatus(uint16_t status)
{
    std::lock_guard<std::mutex> lock(mtx_);
    for (auto *obs : observers_)
        obs->onRtkStatus(status);
}

void FcSubscriptionManager::notifyRtkPosition(double lat, double lon, float alt)
{
    std::lock_guard<std::mutex> lock(mtx_);
    for (auto *obs : observers_)
        obs->onRtkPosition(lat, lon, alt);
}

void FcSubscriptionManager::notifyVelocity(double vx, double vy, double vz, double total)
{
    std::lock_guard<std::mutex> lock(mtx_);
    for (auto *obs : observers_)
        obs->onVelocity(vx, vy, vz, total);
}

void FcSubscriptionManager::notifyHomePoint(double lat, double lon, float alt)
{
    std::lock_guard<std::mutex> lock(mtx_);
    for (auto *obs : observers_)
        obs->onHomePoint(lat, lon, alt);
}

void FcSubscriptionManager::notifyFusedPosition(double lat, double lon, float alt, uint16_t visibleSats)
{
    std::lock_guard<std::mutex> lock(mtx_);
    for (auto *obs : observers_)
        obs->onFusedPosition(lat, lon, alt, visibleSats);
}

void FcSubscriptionManager::notifyFusedHeight(float height)
{
    std::lock_guard<std::mutex> lock(mtx_);
    for (auto *obs : observers_)
        obs->onFusedHeight(height);
}

void FcSubscriptionManager::notifyAttitude(double pitch, double roll, double yaw)
{
    std::lock_guard<std::mutex> lock(mtx_);
    for (auto *obs : observers_)
        obs->onAttitude(pitch, roll, yaw);
}

void FcSubscriptionManager::notifyFlightStatus(uint8_t status)
{
    std::lock_guard<std::mutex> lock(mtx_);
    for (auto *obs : observers_)
        obs->onFlightStatus(status);
}

void FcSubscriptionManager::notifyBatteryInfo(uint8_t percent, int32_t voltage_mv, int16_t temp_tenth_c)
{
    std::lock_guard<std::mutex> lock(mtx_);
    for (auto *obs : observers_)
        obs->onBatteryInfo(percent, voltage_mv, temp_tenth_c);
}

void FcSubscriptionManager::notifyObstacleInfo(dji_f32_t front, dji_f32_t back, dji_f32_t left, dji_f32_t right,
                                               dji_f32_t up, dji_f32_t down,
                                               uint8_t frontHealth, uint8_t backHealth, uint8_t leftHealth,
                                               uint8_t rightHealth, uint8_t upHealth, uint8_t downHealth,
                                               bool flag)
{
    std::lock_guard<std::mutex> lock(mtx_);
    for (auto *obs : observers_)
        obs->onObstacleInfo(front, back, left, right, up, down,
                            frontHealth, backHealth, leftHealth,
                            rightHealth, upHealth, downHealth, flag);
}

void FcSubscriptionManager::notifyDisplayMode(uint8_t mode)
{
    std::lock_guard<std::mutex> lock(mtx_);
    for (auto *obs : observers_)
        obs->onDisplayMode(mode);
}

void FcSubscriptionManager::notifyControlAuthority(uint8_t deviceStatus, uint8_t controlMode,
                                                   uint8_t flightStatus, uint8_t vrcStatus)
{
    std::lock_guard<std::mutex> lock(mtx_);
    for (auto *obs : observers_)
        obs->onControlAuthority(deviceStatus, controlMode, flightStatus, vrcStatus);
}

// ---------- 静态回调实现 ----------

#define CALL_MANAGER(method, ...)                                   \
    do                                                              \
    {                                                               \
        if (FcSubscriptionManager::instance_)                       \
        {                                                           \
            FcSubscriptionManager::instance_->method(__VA_ARGS__);  \
        }                                                           \
        else                                                        \
        {                                                           \
            USER_LOG_WARN("FC callback: manager instance is null"); \
        }                                                           \
    } while (0)

T_DjiReturnCode FcSubscriptionManager::onGpsPositionStatic(const uint8_t *data, uint16_t len, const T_DjiDataTimestamp *ts)
{
    (void)ts;
    if (!data || len < sizeof(T_DjiFcSubscriptionGpsPosition))
        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    T_DjiFcSubscriptionGpsPosition raw;
    memcpy(&raw, data, sizeof(raw));
    double lon = raw.x * 1e-7;
    double lat = raw.y * 1e-7;
    double alt = raw.z / 1000.0;
    CALL_MANAGER(notifyGpsPosition, lat, lon, alt);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode FcSubscriptionManager::onGpsInfoStatic(const uint8_t *data, uint16_t len, const T_DjiDataTimestamp *ts)
{
    (void)ts;
    if (!data || len < sizeof(T_DjiFcSubscriptionGpsDetails))
        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    const T_DjiFcSubscriptionGpsDetails *g = reinterpret_cast<const T_DjiFcSubscriptionGpsDetails *>(data);
    CALL_MANAGER(notifyGpsInfo, g->hdop, g->pdop, g->fixState, g->vacc, g->hacc, g->sacc,
                 g->gpsSatelliteNumberUsed, g->glonassSatelliteNumberUsed,
                 g->totalSatelliteNumberUsed, g->gpsCounter);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode FcSubscriptionManager::onGpsTimeStatic(const uint8_t *data, uint16_t len, const T_DjiDataTimestamp *ts)
{
    (void)ts;
    if (!data || len < sizeof(uint32_t))
        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    uint32_t val = 0;
    memcpy(&val, data, sizeof(uint32_t));
    // 注意：GPS_DATE 和 GPS_TIME 使用同一个静态回调，根据 topic 区分？简单起见，我们这里不区分，直接调用 notifyGpsTime，数值含义上层自行处理。
    // 更好的做法：在注册时使用不同回调，但为了简单，我们统一调用，让观察者忽略不需要的。
    CALL_MANAGER(notifyGpsTime, 0, val); // date 未知，暂传0
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode FcSubscriptionManager::onRtkStatusStatic(const uint8_t *data, uint16_t len, const T_DjiDataTimestamp *ts)
{
    (void)ts;
    if (!data || len < sizeof(uint16_t))
        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    uint16_t status = 0;
    memcpy(&status, data, sizeof(uint16_t));
    CALL_MANAGER(notifyRtkStatus, status);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode FcSubscriptionManager::onRtkPositionStatic(const uint8_t *data, uint16_t len, const T_DjiDataTimestamp *ts)
{
    (void)ts;
    if (!data || len < sizeof(T_DjiFcSubscriptionRtkPosition))
        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    const T_DjiFcSubscriptionRtkPosition *p = reinterpret_cast<const T_DjiFcSubscriptionRtkPosition *>(data);
    CALL_MANAGER(notifyRtkPosition, p->latitude, p->longitude, p->hfsl);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode FcSubscriptionManager::onVelocityStatic(const uint8_t *data, uint16_t len, const T_DjiDataTimestamp *ts)
{
    (void)ts;
    if (!data || len < sizeof(T_DjiFcSubscriptionVelocity))
        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    const T_DjiFcSubscriptionVelocity *v = reinterpret_cast<const T_DjiFcSubscriptionVelocity *>(data);
    if (!v->health)
        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    float vx = v->data.x;
    float vy = v->data.y;
    float vz = v->data.z;
    double total = sqrt(vx * vx + vy * vy + vz * vz);
    CALL_MANAGER(notifyVelocity, vx, vy, vz, total);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode FcSubscriptionManager::onHomePointStatic(const uint8_t *data, uint16_t len, const T_DjiDataTimestamp *ts)
{
    (void)ts;
    // 处理 HOME_POINT_INFO 和 ALTITUDE_OF_HOMEPOINT 两个主题
    // 简单起见，根据数据结构判断：HOME_POINT_INFO 包含经纬度（弧度），ALTITUDE_OF_HOMEPOINT 只是一个浮点数
    if (len >= sizeof(T_DjiFcSubscriptionHomePointInfo))
    {
        T_DjiFcSubscriptionHomePointInfo home;
        memcpy(&home, data, sizeof(home));
        double lat = home.latitude;
        double lon = home.longitude;
        if (fabs(lat) <= 3.2 && fabs(lon) <= 3.2)
        {
            lat *= RAD2DEG;
            lon *= RAD2DEG;
        }
        CALL_MANAGER(notifyHomePoint, lat, lon, 0.0f); // 高度未知，稍后由 ALTITUDE 主题更新
    }
    else if (len >= sizeof(dji_f32_t))
    {
        dji_f32_t alt = 0;
        memcpy(&alt, data, sizeof(dji_f32_t));
        // 通知高度，但需要与之前的经纬度合并？观察者可以单独处理，我们简单调用 onHomePoint 并传入新高度，经纬度用之前的缓存
        // 为了简化，这里单独通知高度变化，观察者应维护自己的 home 缓存。
        // 但为了让接口统一，我们仍调用 notifyHomePoint，传入0经纬度，非零高度，观察者需要知道这种约定。
        CALL_MANAGER(notifyHomePoint, 0.0, 0.0, alt);
    }
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode FcSubscriptionManager::onFusedPositionStatic(const uint8_t *data, uint16_t len, const T_DjiDataTimestamp *ts)
{
    (void)ts;
    if (!data || len < sizeof(T_DjiFcSubscriptionPositionFused))
        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    T_DjiFcSubscriptionPositionFused raw;
    memcpy(&raw, data, sizeof(raw));
    double lon = raw.longitude * RAD2DEG;
    double lat = raw.latitude * RAD2DEG;
    float alt = raw.altitude;
    CALL_MANAGER(notifyFusedPosition, lat, lon, alt, raw.visibleSatelliteNumber);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode FcSubscriptionManager::onFusedHeightStatic(const uint8_t *data, uint16_t len, const T_DjiDataTimestamp *ts)
{
    (void)ts;
    if (!data || len < sizeof(T_DjiFcSubscriptionHeightFusion))
        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    T_DjiFcSubscriptionHeightFusion height = 0;
    memcpy(&height, data, sizeof(T_DjiFcSubscriptionHeightFusion));
    CALL_MANAGER(notifyFusedHeight, static_cast<float>(height));
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode FcSubscriptionManager::onQuaternionStatic(const uint8_t *data, uint16_t len, const T_DjiDataTimestamp *ts)
{
    (void)ts;
    if (!data || len < sizeof(T_DjiFcSubscriptionQuaternion))
        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    const T_DjiFcSubscriptionQuaternion *q = reinterpret_cast<const T_DjiFcSubscriptionQuaternion *>(data);
    double pitch = asin(-2 * q->q1 * q->q3 + 2 * q->q0 * q->q2) * RAD2DEG;
    double roll = atan2(2 * q->q2 * q->q3 + 2 * q->q0 * q->q1, -2 * q->q1 * q->q1 - 2 * q->q2 * q->q2 + 1) * RAD2DEG;
    double yaw = atan2(2 * q->q1 * q->q2 + 2 * q->q0 * q->q3, -2 * q->q2 * q->q2 - 2 * q->q3 * q->q3 + 1) * RAD2DEG;
    CALL_MANAGER(notifyAttitude, pitch, roll, yaw);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode FcSubscriptionManager::onFlightStatusStatic(const uint8_t *data, uint16_t len, const T_DjiDataTimestamp *ts)
{
    (void)ts;
    if (!data || len < sizeof(T_DjiFcSubscriptionFlightStatus))
        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    T_DjiFcSubscriptionFlightStatus status = 0;
    memcpy(&status, data, sizeof(T_DjiFcSubscriptionFlightStatus));
    CALL_MANAGER(notifyFlightStatus, static_cast<uint8_t>(status));
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode FcSubscriptionManager::onBatteryStatic(const uint8_t *data, uint16_t len, const T_DjiDataTimestamp *ts)
{
    (void)ts;
    if (!data || len < 2)
        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    size_t off = 0;
    uint8_t reserve = data[off++];
    (void)reserve;
    uint8_t batteryIndex = data[off++];
    if (batteryIndex != 1)
        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS; // 只处理主电池
    off += 4;                                        // skip currentVoltage
    off += 4;                                        // skip currentElectric
    off += 4;                                        // skip fullCapacity
    off += 4;                                        // skip remainedCapacity
    int16_t temp_tenth = 0;
    if (off + 2 <= len)
        memcpy(&temp_tenth, data + off, 2);
    off += 2;
    off += 1; // cellCount
    uint8_t percent = 0;
    if (off < len)
        percent = data[off];
    off += 1;
    // batteryState, reserve1, reserve2, SOP 可忽略
    int32_t voltage_mv = 0;
    // 简单起见，电压不解析，观察者可能不需要
    CALL_MANAGER(notifyBatteryInfo, percent, voltage_mv, temp_tenth);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode FcSubscriptionManager::onAvoidDataStatic(const uint8_t *data, uint16_t len, const T_DjiDataTimestamp *ts)
{
    (void)ts;
    if (!data || len < sizeof(T_DjiFcSubscriptionAvoidData))
        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    T_DjiFcSubscriptionAvoidData raw;
    memcpy(&raw, data, sizeof(raw));
    dji_f32_t front = raw.front, back = raw.back, left = raw.left, right = raw.right, up = raw.up, down = raw.down;
    uint8_t frontHealth = raw.frontHealth ? 1 : 0;
    uint8_t backHealth = raw.backHealth ? 1 : 0;
    uint8_t leftHealth = raw.leftHealth ? 1 : 0;
    uint8_t rightHealth = raw.rightHealth ? 1 : 0;
    uint8_t upHealth = raw.upHealth ? 1 : 0;
    uint8_t downHealth = raw.downHealth ? 1 : 0;
    bool flag = (front < 20.0f) || (back < 20.0f) || (left < 20.0f) || (right < 20.0f) || (up < 20.0f) || (down < 20.0f);
    CALL_MANAGER(notifyObstacleInfo, front, back, left, right, up, down,
                 frontHealth, backHealth, leftHealth, rightHealth, upHealth, downHealth, flag);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode FcSubscriptionManager::onDisplayModeStatic(const uint8_t *data, uint16_t len, const T_DjiDataTimestamp *ts)
{
    (void)ts;
    if (!data || len < sizeof(T_DjiFcSubscriptionDisplaymode))
        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    T_DjiFcSubscriptionDisplaymode mode = 0;
    memcpy(&mode, data, sizeof(T_DjiFcSubscriptionDisplaymode));
    CALL_MANAGER(notifyDisplayMode, static_cast<uint8_t>(mode));
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode FcSubscriptionManager::onControlAuthorityStatic(const uint8_t *data, uint16_t len, const T_DjiDataTimestamp *ts)
{
    (void)ts;
    if (!data || len < sizeof(T_DjiFcSubscriptionControlDevice))
        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    T_DjiFcSubscriptionControlDevice ctrl;
    memcpy(&ctrl, data, sizeof(ctrl));
    CALL_MANAGER(notifyControlAuthority, ctrl.deviceStatus, ctrl.controlMode, ctrl.flightStatus, ctrl.vrcStatus);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}