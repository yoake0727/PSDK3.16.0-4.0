/**
 * @file telemetry_pos_publisher.cpp
 * @brief 位置/姿态高频发布器实现
 *
 * 职责：
 *   1. 接收飞控回调（IFcDataObserver）的位置/高度/姿态，写入缓存
 *   2. 以固定频率把缓存 + 妙算3系统时间组装成 JSON，通过 MQTT 发布
 *   3. 同时把遥测推给 Localizer，供像素->GPS 的时间对齐使用
 *   4. 统一时间基准：全部使用系统时间（system_clock），不做 NTP
 */

#include "telemetry_pos_publisher.hpp"
#include "dji_logger.h"
#include "3rdparty/json.hpp"

#include <chrono>
#include <thread>
#include <ctime>
#include <cstdio>

#include <unistd.h>

using json = nlohmann::json;
using ordered_json = nlohmann::ordered_json;

// --------------------------------------------------------------------------
/**
 * 构造：只保存 MQTT 桥的引用，不启动线程。
 */
TelemetryPosPublisher::TelemetryPosPublisher(MqttBridge &mqtt) : mqtt_(mqtt) {}

/**
 * 析构：自动 stop()，保证线程退出。
 */
TelemetryPosPublisher::~TelemetryPosPublisher() { stop(); }

// --------------------------------------------------------------------------
/**
 * 启动发布器：只启动发布线程。
 *
 * 幂等：running_ 用 exchange 保证只启动一次。
 */
void TelemetryPosPublisher::start()
{
    if (running_.exchange(true))
        return;

    pub_thread_ = std::thread(&TelemetryPosPublisher::runLoop, this);
}

/**
 * 停止发布器：
 *   1. 置 running_ = false
 *   2. join 发布线程
 *
 * 幂等：running_ 已为 false 时直接返回。
 */
void TelemetryPosPublisher::stop()
{
    if (!running_.exchange(false))
        return;
    if (pub_thread_.joinable())
        pub_thread_.join();
    USER_LOG_INFO("TelemetryPosPublisher stopped");
}

// --------------------------------------------------------------------------
/**
 * 系统时钟（墙钟）毫秒时间戳。
 * 妙算3全机统一时间基准：帧时间戳、遥测样本、发布时间都用它。
 */
uint64_t TelemetryPosPublisher::nowMs()
{
    auto now = std::chrono::system_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
}

// --------------------------------------------------------------------------
/**
 * 发布线程主循环：每 PUBLISH_INTERVAL_MS 发一次。
 * 不做其他事，逻辑都在 publishPos() 里。
 */
void TelemetryPosPublisher::runLoop()
{
    while (running_)
    {
        publishPos();
        std::this_thread::sleep_for(std::chrono::milliseconds(PUBLISH_INTERVAL_MS));
    }
}

// --------------------------------------------------------------------------
/**
 * 组装并发布一次位置/姿态 JSON。
 *
 * 流程：
 *   1. 加锁快照缓存（避免持锁做网络/序列化）
 *   2. 计算系统时间（毫秒 + 可读字符串）
 *   3. 组装 ordered_json（保持字段顺序，便于阅读）
 *   4. 发布到 drone/<drone_id>/psdk/telemetry/pos，QoS=1，retain=true
 *
 * 注意：
 *   - 时间戳用妙算3系统时间（与帧时间戳同源）
 *   - fused_height 是相对地面高度，fused_alt 是海拔
 *   - 异常被捕获，不让 JSON 错误杀掉发布线程
 */
void TelemetryPosPublisher::publishPos()
{
    // 1. 快照缓存
    PosCache snap;
    {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        snap = cache_;
    }

    // 2. 系统时间（毫秒，妙算3统一时间基准）
    uint64_t ts_ms = nowMs();

    // 3. 构建可读时间字符串 (UTC)
    char ts_time_str[50] = {0};
    time_t sec_part = static_cast<time_t>(ts_ms / 1000);
    int ms_part = static_cast<int>(ts_ms % 1000);
    struct tm tm_buf;
    gmtime_r(&sec_part, &tm_buf); // UTC，如需本地时间改为 localtime_r
    char time_str[40];
    strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", &tm_buf);
    snprintf(ts_time_str, sizeof(ts_time_str), "%s.%03d", time_str, ms_part);

    // 4. 组装 JSON
    try
    {
        ordered_json j;
        j["ntp_time"] = ts_ms;             // 兼容保留字段名，实为妙算3系统时间（毫秒）
        j["ntp_time_str"] = ts_time_str;   // 可读 UTC 字符串
        j["ntp_synced"] = true;            // 统一系统时间，恒为真

        j["fused_position"]["fused_lat"] = snap.fused_lat;
        j["fused_position"]["fused_lon"] = snap.fused_lon;
        j["fused_position"]["fused_alt"] = snap.fused_alt;
        j["fused_position"]["fused_height"] = snap.fused_height; // 相对地面高度

        j["attitude"]["pitch"] = snap.pitch;
        j["attitude"]["roll"] = snap.roll;
        j["attitude"]["yaw"] = snap.yaw;

        // 5. 发布到指定主题
        std::string topic = "drone/" + mqtt_.cfg().drone_id + "/psdk/telemetry/pos";
        mqtt_.publish(topic, j.dump(), 1, true); // QOS=1, retain=true
    }
    catch (const std::exception &e)
    {
        USER_LOG_ERROR("publishPos JSON error: %s", e.what());
    }
}

// --------------------------------------------------------------------------
// IFcDataObserver 回调实现
// 三个回调都做同一件事：更新缓存字段，然后把完整快照推给 Localizer
// --------------------------------------------------------------------------

/**
 * 飞控融合位置回调（经纬度 + 海拔）。
 * 更新缓存后推给 Localizer。
 */
void TelemetryPosPublisher::onFusedPosition(double lat, double lon, float alt, uint16_t /*visibleSats*/)
{
    std::lock_guard<std::mutex> lock(cache_mutex_);
    cache_.fused_lat = lat;
    cache_.fused_lon = lon;
    cache_.fused_alt = alt;
    if (localizer_) localizer_->updateTelemetry({nowMs(), cache_.fused_lat, cache_.fused_lon, cache_.fused_height, cache_.pitch, cache_.roll, cache_.yaw});
}

/**
 * 飞控融合高度回调（相对地面高度）。
 * 更新缓存后推给 Localizer。
 */
void TelemetryPosPublisher::onFusedHeight(float height)
{
    std::lock_guard<std::mutex> lock(cache_mutex_);
    cache_.fused_height = height;
    if (localizer_) localizer_->updateTelemetry({nowMs(), cache_.fused_lat, cache_.fused_lon, cache_.fused_height, cache_.pitch, cache_.roll, cache_.yaw});
}

/**
 * 飞控姿态回调（pitch/roll/yaw）。
 * 更新缓存后推给 Localizer。
 */
void TelemetryPosPublisher::onAttitude(double pitch, double roll, double yaw)
{
    std::lock_guard<std::mutex> lock(cache_mutex_);
    cache_.pitch = pitch;
    cache_.roll = roll;
    cache_.yaw = yaw;
    if (localizer_) localizer_->updateTelemetry({nowMs(), cache_.fused_lat, cache_.fused_lon, cache_.fused_height, cache_.pitch, cache_.roll, cache_.yaw});
}

// 为 static constexpr 提供类外定义（避免链接错误）
// C++11 起，static constexpr 成员如果在 ODR-use 场景（如取地址、绑定引用）需要类外定义
constexpr int TelemetryPosPublisher::PUBLISH_INTERVAL_MS;
