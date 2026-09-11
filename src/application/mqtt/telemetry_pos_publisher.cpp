/**
 * @file telemetry_pos_publisher.cpp
 * @brief 位置/姿态高频发布器实现
 *
 * 职责：
 *   1. 接收飞控回调（IFcDataObserver）的位置/高度/姿态，写入缓存
 *   2. 以固定频率把缓存 + NTP 时间组装成 JSON，通过 MQTT 发布
 *   3. 后台线程周期性做 NTP 同步，为发布的时间戳提供真实 UTC
 *   4. 同时把遥测推给 Localizer，供像素->GPS 的时间对齐使用
 */

#include "telemetry_pos_publisher.hpp"
#include "dji_logger.h"
#include "3rdparty/json.hpp"

#include <chrono>
#include <thread>
#include <cmath>
#include <cstring>
#include <cerrno>
#include <ctime>
#include <cstdio>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <sys/time.h>

using json = nlohmann::json;
using ordered_json = nlohmann::ordered_json;

// NTP 时间戳起点：1900-01-01 到 1970-01-01 的秒数
// NTP 字段是 1900 起的秒数，Unix 是 1970 起的，换算时减去此值
#define NTP_TIMESTAMP_DELTA 2208988800ull

#define NTP_PORT 123          // 标准 NTP 端口
#define NTP_TIMEOUT_SEC 2     // NTP 请求超时

// --------------------------------------------------------------------------
/**
 * 构造：只保存 MQTT 桥的引用，不启动线程。
 */
TelemetryPosPublisher::TelemetryPosPublisher(MqttBridge &mqtt) : mqtt_(mqtt) {}

/**
 * 析构：自动 stop()，保证线程退出、NTP socket 关闭。
 */
TelemetryPosPublisher::~TelemetryPosPublisher() { stop(); }

// --------------------------------------------------------------------------
/**
 * 启动发布器：启动发布线程 + NTP 同步线程。
 *
 * 幂等：running_ / sync_running_ 用 exchange 保证只启动一次。
 */
void TelemetryPosPublisher::start()
{
    if (running_.exchange(true))
        return;

    pub_thread_ = std::thread(&TelemetryPosPublisher::runLoop, this);

    if (!sync_running_.exchange(true))
    {
        sync_thread_ = std::thread(&TelemetryPosPublisher::syncLoop, this);
    }
}

/**
 * 停止发布器：
 *   1. 置 running_ = false
 *   2. join 发布线程
 *   3. 置 sync_running_ = false
 *   4. join 同步线程
 *
 * 幂等：running_ 已为 false 时直接返回。
 */
void TelemetryPosPublisher::stop()
{
    if (!running_.exchange(false))
        return;
    if (pub_thread_.joinable())
        pub_thread_.join();

    if (sync_running_.exchange(false))
    {
        if (sync_thread_.joinable())
            sync_thread_.join();
    }
    USER_LOG_INFO("TelemetryPosPublisher stopped");
}

// --------------------------------------------------------------------------
/**
 * 系统时钟（墙钟）毫秒时间戳。
 * 可能被 NTP 调整，不适合做时间对齐。
 */
uint64_t TelemetryPosPublisher::nowMs()
{
    auto now = std::chrono::system_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
}

/**
 * 单调时钟毫秒时间戳。
 * 不会被 NTP 调整，适合做时间对齐（Localizer 用的就是这个）。
 */
uint64_t TelemetryPosPublisher::monotonicMs()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
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
 *   2. 计算 NTP 时间（毫秒 + 可读字符串）
 *   3. 组装 ordered_json（保持字段顺序，便于阅读）
 *   4. 发布到 drone/<drone_id>/psdk/telemetry/pos，QoS=1，retain=true
 *
 * 注意：
 *   - 时间戳用 NTP（真实 UTC），不是单调时钟
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

    // 2. NTP 时间（秒，double）
    double ntp_sec = getCurrentNtpTimestamp();
    uint64_t ntp_ms = static_cast<uint64_t>(ntp_sec * 1000.0);

    bool synced;
    {
        std::lock_guard<std::mutex> lock(offset_mutex_);
        synced = ntp_initialized_;
    }

    // 3. 构建可读时间字符串 (UTC)
    char ntp_time_str[50] = {0};
    time_t sec_part = static_cast<time_t>(ntp_sec);
    int ms_part = static_cast<int>((ntp_sec - sec_part) * 1000);
    struct tm tm_buf;
    gmtime_r(&sec_part, &tm_buf); // UTC，如需本地时间改为 localtime_r
    char time_str[40];
    strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", &tm_buf);
    snprintf(ntp_time_str, sizeof(ntp_time_str), "%s.%03d", time_str, ms_part);

    // 4. 组装 JSON
    try
    {
        ordered_json j;
        j["ntp_time"] = ntp_ms;            // NTP 时间（毫秒）
        j["ntp_time_str"] = ntp_time_str;  // 可读 UTC 字符串
        j["ntp_synced"] = synced;          // NTP 是否已同步

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
/**
 * NTP 同步循环：每 SYNC_INTERVAL_SEC 秒同步一次。
 *
 * 成功：更新 ntp_offset_sec_，置 ntp_initialized_ = true
 * 失败：保留上次 offset，记 warn
 *
 * 分段 sleep：每秒检查一次 sync_running_，保证 stop() 能快速退出。
 */
void TelemetryPosPublisher::syncLoop()
{
    while (sync_running_)
    {
        double new_offset;
        if (fetchNtpOffset(ntp_server_, new_offset))
        {
            std::lock_guard<std::mutex> lock(offset_mutex_);
            ntp_offset_sec_ = new_offset;
            ntp_initialized_ = true;
            USER_LOG_INFO("PosPub NTP sync: offset = %.3f ms", new_offset * 1000.0);
        }
        else
        {
            USER_LOG_ERROR("PosPub NTP sync failed, using last offset");
        }

        // 分段 sleep，保证 stop() 能及时唤醒
        for (int i = 0; i < SYNC_INTERVAL_SEC && sync_running_; ++i)
        {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
}

/**
 * 一次 NTP 同步：发请求、收响应、算 offset。
 *
 * NTP 四时间戳算法：
 *   t1 = 客户端发送时刻（单调时钟）
 *   t2 = 服务端接收时刻（NTP 时间戳）
 *   t3 = 服务端发送时刻（NTP 时间戳）
 *   t4 = 客户端接收时刻（单调时钟）
 *
 *   offset = ((t2 - t1) + (t3 - t4)) / 2
 *   delay  = (t4 - t1) - (t3 - t2)
 *
 * 注意：
 *   - 这里只算 offset，不算 delay / 往返延迟
 *   - t1/t4 用 CLOCK_MONOTONIC（不受系统时间调整影响）
 *   - t2/t3 从 NTP 响应包的第 32~47 字节读取
 */
bool TelemetryPosPublisher::fetchNtpOffset(const std::string &server, double &offset)
{
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0)
    {
        USER_LOG_ERROR("PosPub NTP socket error: %s", strerror(errno));
        return false;
    }

    // 设置接收超时
    struct timeval tv = {NTP_TIMEOUT_SEC, 0};
    setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    struct sockaddr_in serv_addr;
    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(NTP_PORT);
    if (inet_pton(AF_INET, server.c_str(), &serv_addr.sin_addr) <= 0)
    {
        USER_LOG_ERROR("PosPub invalid NTP server: %s", server.c_str());
        close(sockfd);
        return false;
    }

    // NTP 请求包 48 字节，首字节 0x1b = LI=0, VN=3, Mode=3（客户端）
    char send_buf[48] = {0};
    send_buf[0] = 0x1b; // NTPv3 client

    struct timespec t1, t4;
    clock_gettime(CLOCK_MONOTONIC, &t1);  // t1：发送前

    if (sendto(sockfd, send_buf, sizeof(send_buf), 0,
               (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    {
        USER_LOG_ERROR("PosPub NTP sendto error: %s", strerror(errno));
        close(sockfd);
        return false;
    }

    char recv_buf[48];
    socklen_t addr_len = sizeof(serv_addr);
    if (recvfrom(sockfd, recv_buf, sizeof(recv_buf), 0,
                 (struct sockaddr *)&serv_addr, &addr_len) < 0)
    {
        USER_LOG_ERROR("PosPub NTP recvfrom timeout/error: %s", strerror(errno));
        close(sockfd);
        return false;
    }
    close(sockfd);

    clock_gettime(CLOCK_MONOTONIC, &t4);  // t4：接收后

    // 从响应包读取 t2（接收时间戳）和 t3（发送时间戳）
    // 偏移 32~39 是 t2，40~47 是 t3，各 4 字节秒 + 4 字节小数
    uint32_t t2_sec, t2_frac, t3_sec, t3_frac;
    memcpy(&t2_sec, &recv_buf[32], 4);
    t2_sec = ntohl(t2_sec);
    memcpy(&t2_frac, &recv_buf[36], 4);
    t2_frac = ntohl(t2_frac);
    memcpy(&t3_sec, &recv_buf[40], 4);
    t3_sec = ntohl(t3_sec);
    memcpy(&t3_frac, &recv_buf[44], 4);
    t3_frac = ntohl(t3_frac);

    // 单调时钟转 double 秒
    double t1_d = t1.tv_sec + t1.tv_nsec / 1e9;
    double t4_d = t4.tv_sec + t4.tv_nsec / 1e9;

    // NTP 时间戳转 Unix 秒：秒部分减 1900~1970 的偏移，小数部分除以 2^32
    double t2_d = (double)(t2_sec - NTP_TIMESTAMP_DELTA) + (double)t2_frac / 4294967296.0;
    double t3_d = (double)(t3_sec - NTP_TIMESTAMP_DELTA) + (double)t3_frac / 4294967296.0;

    // NTP offset 公式
    offset = ((t2_d - t1_d) + (t3_d - t4_d)) / 2.0;
    return true;
}

/**
 * 取当前 NTP 时间戳（Unix 秒，double）。
 *
 * 已同步：单调时钟 + ntp_offset_sec_
 *   用单调时钟做基准，避免系统时间被调整时影响连续性
 * 未同步：回退到系统时间（system_clock）
 *
 * 注意：
 *   - 单调时钟起点是系统启动时刻，不是 Unix epoch，
 *     所以 mono_sec + offset 里的 offset 包含了"启动时刻到 Unix epoch"的差值
 *   - 加锁保护 offset 和 initialized
 */
double TelemetryPosPublisher::getCurrentNtpTimestamp()
{
    struct timespec mono;
    clock_gettime(CLOCK_MONOTONIC, &mono);
    double mono_sec = mono.tv_sec + mono.tv_nsec / 1e9;

    std::lock_guard<std::mutex> lock(offset_mutex_);
    if (!ntp_initialized_)
    {
        // 回退到系统时间
        auto now = std::chrono::system_clock::now();
        auto duration = now.time_since_epoch();
        return std::chrono::duration<double>(duration).count();
    }
    return mono_sec + ntp_offset_sec_;
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
    if (localizer_) localizer_->updateTelemetry({monotonicMs(), cache_.fused_lat, cache_.fused_lon, cache_.fused_height, cache_.pitch, cache_.roll, cache_.yaw});
}

/**
 * 飞控融合高度回调（相对地面高度）。
 * 更新缓存后推给 Localizer。
 */
void TelemetryPosPublisher::onFusedHeight(float height)
{
    std::lock_guard<std::mutex> lock(cache_mutex_);
    cache_.fused_height = height;
    if (localizer_) localizer_->updateTelemetry({monotonicMs(), cache_.fused_lat, cache_.fused_lon, cache_.fused_height, cache_.pitch, cache_.roll, cache_.yaw});
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
    if (localizer_) localizer_->updateTelemetry({monotonicMs(), cache_.fused_lat, cache_.fused_lon, cache_.fused_height, cache_.pitch, cache_.roll, cache_.yaw});
}

// 为 static constexpr 提供类外定义（避免链接错误）
// C++11 起，static constexpr 成员如果在 ODR-use 场景（如取地址、绑定引用）需要类外定义
constexpr int TelemetryPosPublisher::PUBLISH_INTERVAL_MS;
constexpr int TelemetryPosPublisher::SYNC_INTERVAL_SEC;
