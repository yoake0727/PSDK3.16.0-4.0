/**
 * @file telemetry_pos_publisher.cpp
 * @brief 位置/姿态高频发布器实现
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

#define NTP_TIMESTAMP_DELTA 2208988800ull
#define NTP_PORT 123
#define NTP_TIMEOUT_SEC 2

// --------------------------------------------------------------------------
TelemetryPosPublisher::TelemetryPosPublisher(MqttBridge &mqtt) : mqtt_(mqtt) {}

TelemetryPosPublisher::~TelemetryPosPublisher() { stop(); }

// --------------------------------------------------------------------------
void TelemetryPosPublisher::start()
{
    if (running_.exchange(true))
        return;

    pub_thread_ = std::thread(&TelemetryPosPublisher::runLoop, this);

    if (!sync_running_.exchange(true))
    {
        sync_thread_ = std::thread(&TelemetryPosPublisher::syncLoop, this);
    }

    USER_LOG_INFO("TelemetryPosPublisher started (50Hz + NTP sync)");
}

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
uint64_t TelemetryPosPublisher::nowMs()
{
    auto now = std::chrono::system_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
}

// --------------------------------------------------------------------------
void TelemetryPosPublisher::runLoop()
{
    while (running_)
    {
        publishPos();
        std::this_thread::sleep_for(std::chrono::milliseconds(PUBLISH_INTERVAL_MS));
    }
}

// --------------------------------------------------------------------------
void TelemetryPosPublisher::publishPos()
{
    // 1. 快照缓存
    PosCache snap;
    {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        snap = cache_;
    }

    // 2. NTP 时间
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
        j["ntp_time"] = ntp_ms;
        j["ntp_time_str"] = ntp_time_str;
        j["ntp_synced"] = synced;

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
// NTP 同步循环
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
            USER_LOG_WARN("PosPub NTP sync failed, using last offset");
        }

        for (int i = 0; i < SYNC_INTERVAL_SEC && sync_running_; ++i)
        {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
}

bool TelemetryPosPublisher::fetchNtpOffset(const std::string &server, double &offset)
{
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0)
    {
        USER_LOG_ERROR("PosPub NTP socket error: %s", strerror(errno));
        return false;
    }

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

    char send_buf[48] = {0};
    send_buf[0] = 0x1b; // NTPv3 client

    struct timespec t1, t4;
    clock_gettime(CLOCK_MONOTONIC, &t1);

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

    clock_gettime(CLOCK_MONOTONIC, &t4);

    uint32_t t2_sec, t2_frac, t3_sec, t3_frac;
    memcpy(&t2_sec, &recv_buf[32], 4);
    t2_sec = ntohl(t2_sec);
    memcpy(&t2_frac, &recv_buf[36], 4);
    t2_frac = ntohl(t2_frac);
    memcpy(&t3_sec, &recv_buf[40], 4);
    t3_sec = ntohl(t3_sec);
    memcpy(&t3_frac, &recv_buf[44], 4);
    t3_frac = ntohl(t3_frac);

    double t1_d = t1.tv_sec + t1.tv_nsec / 1e9;
    double t4_d = t4.tv_sec + t4.tv_nsec / 1e9;
    double t2_d = (double)(t2_sec - NTP_TIMESTAMP_DELTA) + (double)t2_frac / 4294967296.0;
    double t3_d = (double)(t3_sec - NTP_TIMESTAMP_DELTA) + (double)t3_frac / 4294967296.0;

    offset = ((t2_d - t1_d) + (t3_d - t4_d)) / 2.0;
    return true;
}

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
void TelemetryPosPublisher::onFusedPosition(double lat, double lon, float alt, uint16_t /*visibleSats*/)
{
    std::lock_guard<std::mutex> lock(cache_mutex_);
    cache_.fused_lat = lat;
    cache_.fused_lon = lon;
    cache_.fused_alt = alt;
}

void TelemetryPosPublisher::onFusedHeight(float height)
{
    std::lock_guard<std::mutex> lock(cache_mutex_);
    cache_.fused_height = height;
}

void TelemetryPosPublisher::onAttitude(double pitch, double roll, double yaw)
{
    std::lock_guard<std::mutex> lock(cache_mutex_);
    cache_.pitch = pitch;
    cache_.roll = roll;
    cache_.yaw = yaw;
}
// 为 static constexpr 提供类外定义（避免链接错误）
constexpr int TelemetryPosPublisher::PUBLISH_INTERVAL_MS;
constexpr int TelemetryPosPublisher::SYNC_INTERVAL_SEC;