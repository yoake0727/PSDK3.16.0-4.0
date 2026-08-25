/**
 * @file telemetry_publisher.cpp
 * @brief 遥测数据发布器实现
 */

#include "telemetry_publisher.hpp"
#include "mqtt/mqtt_bridge.hpp"
#include "dji_logger.h"
#include "3rdparty/json.hpp"

#include <chrono>
#include <thread>
#include <cmath>

using json = nlohmann::json;
using ordered_json = nlohmann::ordered_json;

TelemetryPublisher::TelemetryPublisher(MqttBridge &mqtt) : mqtt_(mqtt) {}

TelemetryPublisher::~TelemetryPublisher()
{
    stop();
}

uint64_t TelemetryPublisher::nowMs()
{
    auto now = std::chrono::system_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
}

void TelemetryPublisher::start_5hz()
{
    if (running_.exchange(true))
        return;
    publish_thread_ = std::thread(&TelemetryPublisher::runLoop, this);
    USER_LOG_INFO("TelemetryPublisher started (5Hz)");
}

void TelemetryPublisher::stop()
{
    if (!running_.exchange(false))
        return;
    if (publish_thread_.joinable())
        publish_thread_.join();
    USER_LOG_INFO("TelemetryPublisher stopped");
}

void TelemetryPublisher::setWaypointFlightId(const std::string &id)
{
    std::lock_guard<std::mutex> lock(cache_mutex_);
    cache_.waypoint_flightid = id;
    USER_LOG_DEBUG("TelemetryPublisher flightId set to %s", id.c_str());
}

void TelemetryPublisher::setHasJoystickControl(bool has)
{
    std::lock_guard<std::mutex> lock(cache_mutex_);
    cache_.has_joystick_control = has;
}

// ---------- runLoop 和 publishSnapshot ----------

void TelemetryPublisher::runLoop()
{
    while (running_)
    {
        publishSnapshot();
        std::this_thread::sleep_for(std::chrono::milliseconds(200)); // 5Hz
    }
}

void TelemetryPublisher::publishSnapshot()
{
    // 快照缓存数据
    TelemetryCache snap;
    {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        snap = cache_;
        // 更新时间戳为当前系统时间（与原版一致）
        snap.ts_utc = nowMs();
    }

    try
    {
        ordered_json j;

        // 严格按照原版 publish_DjiFcSub 的顺序和字段名
        j["ts_utc"] = snap.ts_utc;

        j["gps_time"]["date"] = snap.gps_date;
        j["gps_time"]["time"] = snap.gps_time;

        j["gps_info"]["hdop"] = snap.hdop;
        j["gps_info"]["pdop"] = snap.pdop;
        j["gps_info"]["fixState"] = snap.fixState;
        j["gps_info"]["vacc"] = snap.vacc;
        j["gps_info"]["hacc"] = snap.hacc;
        j["gps_info"]["sacc"] = snap.sacc;
        j["gps_info"]["gpsSatelliteNumberUsed"] = snap.gps_sat_used;
        j["gps_info"]["glonassSatelliteNumberUsed"] = snap.glonass_sat_used;
        j["gps_info"]["totalSatelliteNumberUsed"] = snap.total_sat_used;
        j["gps_info"]["visible_sats"] = snap.visible_sats; // 注意：原版中 visible_sats 在 gps_info 内
        j["gps_info"]["gpsCounter"] = snap.gps_counter;

        j["gps_position"]["gps_lat"] = snap.gps_lat;
        j["gps_position"]["gps_lon"] = snap.gps_lon;
        j["gps_position"]["gps_alt"] = snap.gps_alt;

        j["rtk_position"]["rtk_status"] = snap.rtk_status;
        j["rtk_position"]["rtk_lat"] = snap.rtk_lat;
        j["rtk_position"]["rtk_lon"] = snap.rtk_lon;
        j["rtk_position"]["rtk_alt"] = snap.rtk_alt;

        j["homepointset"]["home_lat"] = snap.home_lat;
        j["homepointset"]["home_lon"] = snap.home_lon;
        j["homepointset"]["home_alt"] = snap.home_alt;

        j["fused_position"]["fused_lat"] = snap.fused_lat;
        j["fused_position"]["fused_lon"] = snap.fused_lon;
        j["fused_position"]["fused_alt"] = snap.fused_alt;

        j["fused_height"] = snap.fused_height;

        j["velocity"]["vel_x"] = snap.vel_x;
        j["velocity"]["vel_y"] = snap.vel_y;
        j["velocity"]["vel_z"] = snap.vel_z;
        j["velocity"]["vel_total"] = snap.vel_total;

        j["attitude"]["pitch"] = snap.pitch;
        j["attitude"]["roll"] = snap.roll;
        j["attitude"]["yaw"] = snap.yaw;

        j["flight_status"] = snap.flight_status;

        j["battery_info"]["percent"] = snap.battery_percent;
        j["battery_info"]["voltage_v"] = snap.battery_voltage_mv;       // 注意：原版字段名 voltage_v，但存储的是毫伏
        j["battery_info"]["temp_c"] = snap.battery_temp_tenth_c / 10.0; // 原版 temp_c 是摄氏度（实际存储为摄氏度？原版注释是 temp_c 但来自 int16_t 0.1°C，需要转换）

        j["obstacle_data"]["front"] = snap.obs_front;
        j["obstacle_data"]["back"] = snap.obs_back;
        j["obstacle_data"]["left"] = snap.obs_left;
        j["obstacle_data"]["right"] = snap.obs_right;
        j["obstacle_data"]["up"] = snap.obs_up;
        j["obstacle_data"]["down"] = snap.obs_down;
        j["obstacle_data"]["frontHealth"] = snap.obs_front_health;
        j["obstacle_data"]["backHealth"] = snap.obs_back_health;
        j["obstacle_data"]["leftHealth"] = snap.obs_left_health;
        j["obstacle_data"]["rightHealth"] = snap.obs_right_health;
        j["obstacle_data"]["upHealth"] = snap.obs_up_health;
        j["obstacle_data"]["downHealth"] = snap.obs_down_health;
        j["obstacle_data"]["flag"] = snap.obs_flag;

        j["flightId"] = snap.waypoint_flightid;

        // 原版中 device_status 赋值为 flight_status (snap.flight_status)
        j["device_status"] = snap.flight_status;

        j["has_joystickControl"] = snap.has_joystick_control ? 1 : 0;

        // 发布主题（与原版相同）
        std::string topic = "drone/" + mqtt_.cfg().drone_id + "/psdk/telemetry/fcsub";
        mqtt_.publish(topic, j.dump(), 1, true); // retain = true
    }
    catch (const std::exception &e)
    {
        USER_LOG_ERROR("publishSnapshot JSON build failed: %s", e.what());
    }
}

// ---------- IFcDataObserver 接口实现 ----------

void TelemetryPublisher::onGpsPosition(double lat, double lon, double alt)
{
    std::lock_guard<std::mutex> lock(cache_mutex_);
    cache_.gps_lat = lat;
    cache_.gps_lon = lon;
    cache_.gps_alt = alt;
}

void TelemetryPublisher::onGpsInfo(dji_f32_t hdop, dji_f32_t pdop, dji_f32_t fixState,
                                   dji_f32_t vacc, dji_f32_t hacc, dji_f32_t sacc,
                                   uint32_t gpsSatUsed, uint32_t glonassSatUsed,
                                   uint32_t totalSatUsed, uint32_t gpsCounter)
{
    std::lock_guard<std::mutex> lock(cache_mutex_);
    cache_.hdop = hdop;
    cache_.pdop = pdop;
    cache_.fixState = fixState;
    cache_.vacc = vacc;
    cache_.hacc = hacc;
    cache_.sacc = sacc;
    cache_.gps_sat_used = gpsSatUsed;
    cache_.glonass_sat_used = glonassSatUsed;
    cache_.total_sat_used = totalSatUsed;
    cache_.gps_counter = gpsCounter;
}

void TelemetryPublisher::onGpsTime(uint32_t date, uint32_t time)
{
    std::lock_guard<std::mutex> lock(cache_mutex_);
    cache_.gps_date = date;
    cache_.gps_time = time;
}

void TelemetryPublisher::onRtkStatus(uint16_t status)
{
    std::lock_guard<std::mutex> lock(cache_mutex_);
    cache_.rtk_status = status;
}

void TelemetryPublisher::onRtkPosition(double lat, double lon, float alt)
{
    std::lock_guard<std::mutex> lock(cache_mutex_);
    cache_.rtk_lat = lat;
    cache_.rtk_lon = lon;
    cache_.rtk_alt = alt;
}

void TelemetryPublisher::onVelocity(double vx, double vy, double vz, double total)
{
    std::lock_guard<std::mutex> lock(cache_mutex_);
    cache_.vel_x = vx;
    cache_.vel_y = vy;
    cache_.vel_z = vz;
    cache_.vel_total = total;
}

void TelemetryPublisher::onHomePoint(double lat, double lon, float alt)
{
    std::lock_guard<std::mutex> lock(cache_mutex_);
    if (lat != 0.0 || lon != 0.0)
    {
        cache_.home_lat = lat;
        cache_.home_lon = lon;
    }
    if (alt != 0.0f)
        cache_.home_alt = alt;
}

void TelemetryPublisher::onFusedPosition(double lat, double lon, float alt, uint16_t visibleSats)
{
    std::lock_guard<std::mutex> lock(cache_mutex_);
    cache_.fused_lat = lat;
    cache_.fused_lon = lon;
    cache_.fused_alt = alt;
    cache_.visible_sats = visibleSats;
}

void TelemetryPublisher::onFusedHeight(float height)
{
    std::lock_guard<std::mutex> lock(cache_mutex_);
    cache_.fused_height = height;
}

void TelemetryPublisher::onAttitude(double pitch, double roll, double yaw)
{
    std::lock_guard<std::mutex> lock(cache_mutex_);
    cache_.pitch = pitch;
    cache_.roll = roll;
    cache_.yaw = yaw;
}

void TelemetryPublisher::onFlightStatus(uint8_t status)
{
    std::lock_guard<std::mutex> lock(cache_mutex_);
    cache_.flight_status = status;
}

void TelemetryPublisher::onBatteryInfo(uint8_t percent, int32_t voltage_mv, int16_t temp_tenth_c)
{
    std::lock_guard<std::mutex> lock(cache_mutex_);
    cache_.battery_percent = percent;
    cache_.battery_voltage_mv = voltage_mv;
    cache_.battery_temp_tenth_c = temp_tenth_c;
}

void TelemetryPublisher::onObstacleInfo(dji_f32_t front, dji_f32_t back, dji_f32_t left, dji_f32_t right,
                                        dji_f32_t up, dji_f32_t down,
                                        uint8_t frontHealth, uint8_t backHealth, uint8_t leftHealth,
                                        uint8_t rightHealth, uint8_t upHealth, uint8_t downHealth,
                                        bool flag)
{
    std::lock_guard<std::mutex> lock(cache_mutex_);
    cache_.obs_front = front;
    cache_.obs_back = back;
    cache_.obs_left = left;
    cache_.obs_right = right;
    cache_.obs_up = up;
    cache_.obs_down = down;
    cache_.obs_front_health = frontHealth;
    cache_.obs_back_health = backHealth;
    cache_.obs_left_health = leftHealth;
    cache_.obs_right_health = rightHealth;
    cache_.obs_up_health = upHealth;
    cache_.obs_down_health = downHealth;
    cache_.obs_flag = flag;
}

void TelemetryPublisher::onDisplayMode(uint8_t mode)
{
    std::lock_guard<std::mutex> lock(cache_mutex_);
    cache_.display_mode = mode;
}

void TelemetryPublisher::onControlAuthority(uint8_t deviceStatus, uint8_t controlMode,
                                            uint8_t flightStatus, uint8_t vrcStatus)
{
    std::lock_guard<std::mutex> lock(cache_mutex_);
    cache_.control_device_status = deviceStatus;
    cache_.control_mode = controlMode;
    cache_.control_flight_status = flightStatus;
    cache_.vrc_status = vrcStatus;
}
