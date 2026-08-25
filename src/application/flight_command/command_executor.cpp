/**
 * @file command_executor.cpp
 * @brief 命令执行器实现
 */

#include "command_executor.hpp"
#include "h30t/h30t_stream_controller.hpp"
#include "mqtt/telemetry_publisher.hpp"
#include "dji_logger.h"
#include "dji_platform.h"
#include "dji_flight_controller.h"
#include "dji_waypoint_v3.h"
#include "dji_fc_subscription.h"
#include "3rdparty/json.hpp"

#include <chrono>
#include <thread>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <cstring>
#include <sys/stat.h>

using json = nlohmann::json;

// 辅助 sleep 函数
static void sleep_ms(uint32_t ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

// 静态指针，用于 C 回调（waypoint callbacks）
static CommandExecutor *g_cmd_executor = nullptr;

// C 风格回调函数（注册给 SDK）
static T_DjiReturnCode on_waypoint_mission_state(T_DjiWaypointV3MissionState state)
{
    if (g_cmd_executor)
    {
        g_cmd_executor->publish_waypoint_mission_state(state);
    }
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode on_waypoint_action_state(T_DjiWaypointV3ActionState state)
{
    if (g_cmd_executor)
    {
        g_cmd_executor->publish_waypoint_action_state(state);
    }
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode on_display_mode_cb(const uint8_t *data, uint16_t len, const T_DjiDataTimestamp *ts)
{
    (void)ts;
    if (!g_cmd_executor || !data || len < sizeof(T_DjiFcSubscriptionDisplaymode))
        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    T_DjiFcSubscriptionDisplaymode mode;
    std::memcpy(&mode, data, sizeof(mode));
    g_cmd_executor->publish_display_mode(mode);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

// -------------------- 构造与析构 --------------------
CommandExecutor::CommandExecutor(MqttBridge &mqtt, const std::string &drone_id)
    : mqtt_(mqtt), drone_id_(drone_id)
{
    g_cmd_executor = this;

    // 注册 waypoint 回调
    DjiWaypointV3_RegMissionStateCallback(on_waypoint_mission_state);
    DjiWaypointV3_RegActionStateCallback(on_waypoint_action_state);
    // 订阅显示模式（用于 publish_display_mode）
    DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_DISPLAYMODE,
                                     DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ,
                                     on_display_mode_cb);

    // 启动工作线程
    flight_worker_run_.store(true);
    flight_worker_ = std::thread(&CommandExecutor::FlightWorkerThread, this);
}

CommandExecutor::~CommandExecutor()
{
    g_cmd_executor = nullptr;
    // 停止摇杆持续控制
    StopJoystickHold();
    flight_worker_run_.store(false);
    queue_cv_.notify_all();
    if (flight_worker_.joinable())
        flight_worker_.join();
}

void CommandExecutor::SetTelemetryPublisher(TelemetryPublisher *tp)
{
    telemetry_pub_ = tp;
}

void CommandExecutor::SetH30tStreamController(H30tStreamController *controller)
{
    h30t_stream_controller_ = controller;
    USER_LOG_INFO("H30T stream controller set for CommandExecutor");
}

// -------------------- 入队命令 --------------------
void CommandExecutor::EnqueueFlightCommand(const FlightCommand &cmd)
{
    {
        std::lock_guard<std::mutex> lk(queue_mtx_);
        flight_cmd_queue_.push(cmd);
    }
    queue_cv_.notify_one();
}

void CommandExecutor::HandleMqttCommand(const std::string &topic, const std::string &payload)
{
    USER_LOG_INFO("HandleMqttCommand topic=%s payload=%s", topic.c_str(), payload.c_str());

    json j;
    try
    {
        j = json::parse(payload);
    }
    catch (...)
    {
        USER_LOG_ERROR("Invalid JSON payload");
        publish_ack("unknown", "error", "invalid json");
        return;
    }

    if (!j.contains("cmd") || !j["cmd"].is_string())
    {
        publish_ack("unknown", "error", "missing cmd");
        return;
    }
    std::string cmd = j["cmd"].get<std::string>();
    if (cmd == "h30t_source") {
        if (!h30t_stream_controller_ || !j.contains("source") || !j["source"].is_string()) {
            publish_ack(cmd, "error", "missing source or H30T controller"); return;
        }
        const std::string source = j["source"].get<std::string>();
        H30tSource selected;
        if (source == "wide") selected = H30tSource::kWide;
        else if (source == "zoom") selected = H30tSource::kZoom;
        else if (source == "infrared") selected = H30tSource::kInfrared;
        else { publish_ack(cmd, "error", "source must be wide, zoom, or infrared"); return; }
        if (!h30t_stream_controller_->RequestSource(selected)) publish_ack(cmd, "error", "source request rejected");
        return;
    }

    // 控制权相关
    if (cmd == "obtain_ctrl" || cmd == "obtain_joystick_auth")
    {
        FlightCommand fc;
        fc.type = FlightCommandType::OBTAIN_JOYSTICK_AUTH;
        EnqueueFlightCommand(fc);
        return;
    }
    if (cmd == "release_ctrl" || cmd == "release_joystick_auth")
    {
        FlightCommand fc;
        fc.type = FlightCommandType::RELEASE_JOYSTICK_AUTH;
        EnqueueFlightCommand(fc);
        return;
    }

    // waypoint_upload
    if (cmd == "waypoint_upload")
    {
        if (!j.contains("url") || !j["url"].is_string())
        {
            publish_ack("waypoint_upload", "error", "missing url");
            return;
        }
        FlightCommand fc;
        fc.type = FlightCommandType::WAYPOINT_UPLOAD;
        fc.waypoint_url = j["url"].get<std::string>();
        fc.waypoint_local_path = j.contains("local_path") ? j["local_path"].get<std::string>() : "/tmp/psdk_waypoint.kmz";
        fc.flightid = j.contains("flightId") && j["flightId"].is_string() ? j["flightId"].get<std::string>() : "";
        EnqueueFlightCommand(fc);
        return;
    }

    // waypoint actions
    if (cmd == "waypoint_start" || cmd == "waypoint_pause" || cmd == "waypoint_resume" || cmd == "waypoint_stop")
    {
        FlightCommand fc;
        if (cmd == "waypoint_start")
            fc.type = FlightCommandType::WAYPOINT_START;
        else if (cmd == "waypoint_pause")
            fc.type = FlightCommandType::WAYPOINT_PAUSE;
        else if (cmd == "waypoint_resume")
            fc.type = FlightCommandType::WAYPOINT_RESUME;
        else
            fc.type = FlightCommandType::WAYPOINT_STOP;

        if (j.contains("flightId"))
        {
            if (j["flightId"].is_string())
                fc.flightid = j["flightId"].get<std::string>();
            else if (j["flightId"].is_number())
                fc.flightid = std::to_string(j["flightId"].get<int>());
        }
        EnqueueFlightCommand(fc);
        return;
    }

    // 基本飞行命令
    if (cmd == "takeoff")
    {
        FlightCommand fc;
        fc.type = FlightCommandType::TAKEOFF;
        EnqueueFlightCommand(fc);
        return;
    }
    if (cmd == "start_landing" || cmd == "land")
    {
        FlightCommand fc;
        fc.type = FlightCommandType::LAND;
        EnqueueFlightCommand(fc);
        return;
    }
    if (cmd == "force_landing")
    {
        FlightCommand fc;
        fc.type = FlightCommandType::FORCE_LAND;
        EnqueueFlightCommand(fc);
        return;
    }
    if (cmd == "cancel_landing")
    {
        FlightCommand fc;
        fc.type = FlightCommandType::CANCEL_LAND;
        EnqueueFlightCommand(fc);
        return;
    }
    if (cmd == "start_gohome" || cmd == "go_home")
    {
        FlightCommand fc;
        fc.type = FlightCommandType::GO_HOME;
        EnqueueFlightCommand(fc);
        return;
    }
    if (cmd == "cancel_gohome" || cmd == "cancel_go_home")
    {
        FlightCommand fc;
        fc.type = FlightCommandType::CANCEL_GO_HOME;
        EnqueueFlightCommand(fc);
        return;
    }
    if (cmd == "turn_on_motors" || cmd == "motor_on")
    {
        FlightCommand fc;
        fc.type = FlightCommandType::MOTOR_ON;
        EnqueueFlightCommand(fc);
        return;
    }
    if (cmd == "turn_off_motors" || cmd == "motor_off")
    {
        FlightCommand fc;
        fc.type = FlightCommandType::MOTOR_OFF;
        EnqueueFlightCommand(fc);
        return;
    }
    if (cmd == "emergency_stop")
    {
        FlightCommand fc;
        fc.type = FlightCommandType::EMERGENCY_STOP;
        if (j.contains("reason") && j["reason"].is_string())
            fc.reason = j["reason"].get<std::string>();
        EnqueueFlightCommand(fc);
        return;
    }
    if (cmd == "confirm_landing")
    {
        FlightCommand fc;
        fc.type = FlightCommandType::CONFIRM_LANDING;
        EnqueueFlightCommand(fc);
        return;
    }

    // joystick 单步
    if (cmd == "joystick")
    {
        FlightCommand fc;
        fc.type = FlightCommandType::JOYSTICK_STEP;
        auto sign = [](double v)
        { return (v > 0 ? 1.0 : (v < 0 ? -1.0 : 0.0)); };
        if (j.contains("x") && j["x"].is_number())
            fc.x = sign(j["x"].get<double>());
        if (j.contains("y") && j["y"].is_number())
            fc.y = sign(j["y"].get<double>());
        if (j.contains("z") && j["z"].is_number())
            fc.z = sign(j["z"].get<double>());
        if (j.contains("yaw") && j["yaw"].is_number())
            fc.yaw = sign(j["yaw"].get<double>());
        if (j.contains("hold_ms") && j["hold_ms"].is_number_unsigned())
            fc.hold_ms = j["hold_ms"].get<uint32_t>();
        EnqueueFlightCommand(fc);
        return;
    }

    // joystick hold
    if (cmd == "joystick_hold_start")
    {
        FlightCommand fc;
        fc.type = FlightCommandType::JOYSTICK_HOLD_START;
        auto sign = [](double v)
        { return (v > 0 ? 1.0 : (v < 0 ? -1.0 : 0.0)); };
        if (j.contains("x") && j["x"].is_number())
            fc.x = sign(j["x"].get<double>());
        if (j.contains("y") && j["y"].is_number())
            fc.y = sign(j["y"].get<double>());
        if (j.contains("z") && j["z"].is_number())
            fc.z = sign(j["z"].get<double>());
        if (j.contains("yaw") && j["yaw"].is_number())
            fc.yaw = sign(j["yaw"].get<double>());
        EnqueueFlightCommand(fc);
        return;
    }
    if (cmd == "joystick_hold_stop")
    {
        FlightCommand fc;
        fc.type = FlightCommandType::JOYSTICK_HOLD_STOP;
        EnqueueFlightCommand(fc);
        return;
    }

    publish_ack("unknown", "error", "unsupported cmd");
}

// -------------------- 内部初始化辅助 --------------------
bool CommandExecutor::EnsureFlightControllerInited()
{
    if (fc_inited_.load())
        return true;

    T_DjiFlightControllerRidInfo ridInfo = {0};
    ridInfo.latitude = 22.542812;
    ridInfo.longitude = 113.958902;
    ridInfo.altitude = 0;
    T_DjiReturnCode rc = DjiFlightController_Init(ridInfo);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
        USER_LOG_ERROR("DjiFlightController_Init failed 0x%08llX", rc);
        return false;
    }
    rc = DjiFcSubscription_Init();
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
        USER_LOG_ERROR("DjiFcSubscription_Init failed 0x%08llX", rc);
        return false;
    }

    // 订阅一些必要的主题（如果尚未订阅）
    DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT, DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ, nullptr);
    DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_CONTROL_DEVICE, DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ, nullptr);
    DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_BATTERY_SINGLE_INFO_INDEX1, DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ, nullptr);

    // 初始化 Waypoint V3
    T_DjiReturnCode wrc = DjiWaypointV3_Init();
    if (wrc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
        USER_LOG_WARN("DjiWaypointV3_Init returned 0x%08llX", wrc);
    }

    fc_inited_.store(true);
    USER_LOG_INFO("Flight controller initialized successfully");
    return true;
}

bool CommandExecutor::AcquireJoystickAuthority()
{
    if (has_joystick_auth_.load())
        return true;

    T_DjiReturnCode rc = DjiFlightController_ObtainJoystickCtrlAuthority();
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
        USER_LOG_ERROR("Obtain joystick authority failed 0x%08llX", rc);
        return false;
    }

    sleep_ms(1000);
    has_joystick_auth_.store(true);
    if (telemetry_pub_)
        telemetry_pub_->setHasJoystickControl(true);
    USER_LOG_INFO("Joystick authority acquired");
    return true;
}

bool CommandExecutor::UploadKmzFromUrl(const std::string &url, const std::string &local_path)
{
    USER_LOG_INFO("Download KMZ from %s -> %s", url.c_str(), local_path.c_str());

    if (!EnsureFlightControllerInited())
    {
        USER_LOG_ERROR("Upload failed: FC not inited");
        return false;
    }

    // 创建目录
    size_t pos = local_path.find_last_of('/');
    if (pos != std::string::npos)
    {
        std::string dir = local_path.substr(0, pos);
        std::string mk = "mkdir -p '" + dir + "'";
        system(mk.c_str());
    }

    // 下载
    std::string cmd = "wget -q -O '" + local_path + "' '" + url + "'";
    int rc = system(cmd.c_str());
    if (rc != 0)
    {
        cmd = "curl -s -L -o '" + local_path + "' '" + url + "'";
        rc = system(cmd.c_str());
        if (rc != 0)
        {
            USER_LOG_ERROR("Failed to download KMZ (rc=%d)", rc);
            return false;
        }
    }

    std::ifstream ifs(local_path, std::ios::binary | std::ios::ate);
    if (!ifs)
    {
        USER_LOG_ERROR("Open downloaded KMZ failed");
        return false;
    }
    std::streamsize size = ifs.tellg();
    ifs.seekg(0, std::ios::beg);
    if (size <= 0)
    {
        USER_LOG_ERROR("Downloaded KMZ is empty");
        return false;
    }

    std::vector<uint8_t> buf((size_t)size);
    if (!ifs.read(reinterpret_cast<char *>(buf.data()), size))
    {
        USER_LOG_ERROR("Read KMZ failed");
        return false;
    }

    sleep_ms(100);
    const int max_try = 3;
    for (int attempt = 1; attempt <= max_try; ++attempt)
    {
        T_DjiReturnCode rc2 = DjiWaypointV3_UploadKmzFile(buf.data(), static_cast<uint32_t>(size));
        if (rc2 == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
        {
            USER_LOG_INFO("KMZ upload success on attempt %d", attempt);
            return true;
        }
        USER_LOG_WARN("KMZ upload failed 0x%08llX on attempt %d", rc2, attempt);
        sleep_ms(200 * attempt);
    }
    return false;
}

// -------------------- 摇杆持续控制 --------------------
void CommandExecutor::StartJoystickHold(const FlightCommand &cmd)
{
    {
        std::lock_guard<std::mutex> lk(joystick_hold_mtx_);
        if (joystick_hold_running_.load())
            return;
        joystick_hold_cmd_ = cmd;
        joystick_hold_running_.store(true);
    }

    joystick_hold_thread_ = std::thread([this]()
                                        {
        USER_LOG_INFO("Joystick hold thread started");
        if (!EnsureFlightControllerInited()) {
            publish_ack("joystick_hold", "error", "fc not inited");
            joystick_hold_running_.store(false);
            return;
        }
        if (!has_joystick_auth_.load()) {
            publish_ack("joystick_hold", "error", "no joystick authority");
            joystick_hold_running_.store(false);
            return;
        }

        T_DjiFlightControllerJoystickMode mode = {
            DJI_FLIGHT_CONTROLLER_HORIZONTAL_VELOCITY_CONTROL_MODE,
            DJI_FLIGHT_CONTROLLER_VERTICAL_VELOCITY_CONTROL_MODE,
            DJI_FLIGHT_CONTROLLER_YAW_ANGLE_RATE_CONTROL_MODE,
            DJI_FLIGHT_CONTROLLER_HORIZONTAL_BODY_COORDINATE,
            DJI_FLIGHT_CONTROLLER_STABLE_CONTROL_MODE_ENABLE,
        };
        DjiFlightController_SetJoystickMode(mode);

        FlightCommand localCmd;
        {
            std::lock_guard<std::mutex> lk(joystick_hold_mtx_);
            localCmd = joystick_hold_cmd_;
        }

        T_DjiFlightControllerJoystickCommand jcmd = {0};
        while (joystick_hold_running_.load()) {
            jcmd.x = localCmd.x * joystick_speed_m_s_;
            jcmd.y = localCmd.y * joystick_speed_m_s_;
            jcmd.z = localCmd.z * joystick_speed_m_s_;
            jcmd.yaw = (localCmd.yaw > 0 ? yaw_angle_deg_ : (localCmd.yaw < 0 ? -yaw_angle_deg_ : 0.0));
            DjiFlightController_ExecuteJoystickAction(jcmd);
            sleep_ms(50);
        }

        T_DjiFlightControllerJoystickCommand stop = {0};
        DjiFlightController_ExecuteJoystickAction(stop);
        USER_LOG_INFO("Joystick hold thread stopped"); });
}

void CommandExecutor::StopJoystickHold()
{
    {
        std::lock_guard<std::mutex> lk(joystick_hold_mtx_);
        if (!joystick_hold_running_.load())
            return;
        joystick_hold_running_.store(false);
    }
    if (joystick_hold_thread_.joinable())
        joystick_hold_thread_.join();
}

// -------------------- 工作线程 --------------------
void CommandExecutor::FlightWorkerThread()
{
    while (flight_worker_run_.load())
    {
        FlightCommand cmd;
        {
            std::unique_lock<std::mutex> lk(queue_mtx_);
            if (flight_cmd_queue_.empty())
            {
                queue_cv_.wait_for(lk, std::chrono::milliseconds(200));
                if (!flight_worker_run_.load())
                    break;
                if (flight_cmd_queue_.empty())
                    continue;
            }
            cmd = flight_cmd_queue_.front();
            flight_cmd_queue_.pop();
        }

        switch (cmd.type)
        {
        case FlightCommandType::OBTAIN_JOYSTICK_AUTH:
            if (!EnsureFlightControllerInited())
            {
                publish_ack("obtain_ctrl", "error", "fc not inited");
            }
            else if (AcquireJoystickAuthority())
            {
                publish_ack("obtain_ctrl", "ok", "obtained");
            }
            else
            {
                publish_ack("obtain_ctrl", "error", "failed to obtain authority");
            }
            break;

        case FlightCommandType::RELEASE_JOYSTICK_AUTH:
            if (DjiFlightController_ReleaseJoystickCtrlAuthority() == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
            {
                has_joystick_auth_.store(false);
                if (telemetry_pub_)
                    telemetry_pub_->setHasJoystickControl(false);
                publish_ack("release_ctrl", "ok", "released");
            }
            else
            {
                publish_ack("release_ctrl", "error", "release failed");
            }
            break;

        case FlightCommandType::TAKEOFF:
            if (!EnsureFlightControllerInited())
                publish_ack("takeoff", "error", "fc not inited");
            else if (!has_joystick_auth_.load())
                publish_ack("takeoff", "error", "no joystick auth");
            else if (DjiFlightController_StartTakeoff() == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
                publish_ack("takeoff", "ok", "started");
            else
                publish_ack("takeoff", "error", "start failed");
            break;

        case FlightCommandType::LAND:
            if (!EnsureFlightControllerInited())
                publish_ack("land", "error", "fc not inited");
            else if (DjiFlightController_StartLanding() == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
                publish_ack("land", "ok", "started");
            else
                publish_ack("land", "error", "start failed");
            break;

        case FlightCommandType::FORCE_LAND:
            if (!EnsureFlightControllerInited())
                publish_ack("force_land", "error", "fc not inited");
            else if (DjiFlightController_StartForceLanding() == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
            {
                publish_ack("force_land", "ok", "started");
                cache_.put("current_flightId", "");
                if (telemetry_pub_)
                    telemetry_pub_->setWaypointFlightId("");
            }
            else
                publish_ack("force_land", "error", "start failed");
            break;

        case FlightCommandType::CANCEL_LAND:
            if (!EnsureFlightControllerInited())
                publish_ack("cancel_land", "error", "fc not inited");
            else if (DjiFlightController_CancelLanding() == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
                publish_ack("cancel_land", "ok", "canceled");
            else
                publish_ack("cancel_land", "error", "cancel failed");
            break;

        case FlightCommandType::GO_HOME:
            if (!EnsureFlightControllerInited())
                publish_ack("go_home", "error", "fc not inited");
            else if (DjiFlightController_StartGoHome() == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
            {
                publish_ack("go_home", "ok", "started");
                cache_.put("current_flightId", "");
                if (telemetry_pub_)
                    telemetry_pub_->setWaypointFlightId("");
            }
            else
                publish_ack("go_home", "error", "start failed");
            break;

        case FlightCommandType::CANCEL_GO_HOME:
            if (!EnsureFlightControllerInited())
                publish_ack("cancel_go_home", "error", "fc not inited");
            else if (DjiFlightController_CancelGoHome() == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
                publish_ack("cancel_go_home", "ok", "canceled");
            else
                publish_ack("cancel_go_home", "error", "cancel failed");
            break;

        case FlightCommandType::MOTOR_ON:
            if (DjiFlightController_TurnOnMotors() == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
                publish_ack("motor_on", "ok", "motors on");
            else
                publish_ack("motor_on", "error", "failed");
            break;

        case FlightCommandType::MOTOR_OFF:
            if (DjiFlightController_TurnOffMotors() == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
                publish_ack("motor_off", "ok", "motors off");
            else
                publish_ack("motor_off", "error", "failed");
            break;

        case FlightCommandType::EMERGENCY_STOP:
            if (DjiFlightController_EmergencyStopMotor(DJI_FLIGHT_CONTROLLER_ENABLE_EMERGENCY_STOP_MOTOR, (char *)cmd.reason.c_str()) == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
                publish_ack("emergency_stop", "ok", "stopped");
            else
                publish_ack("emergency_stop", "error", "failed");
            break;

        case FlightCommandType::CONFIRM_LANDING:
            if (!EnsureFlightControllerInited())
                publish_ack("confirm_landing", "error", "fc not inited");
            else if (DjiFlightController_StartConfirmLanding() == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
            {
                publish_ack("confirm_landing", "ok", "confirmed");
                cache_.put("current_flightId", "");
                if (telemetry_pub_)
                    telemetry_pub_->setWaypointFlightId("");
            }
            else
                publish_ack("confirm_landing", "error", "failed");
            break;

        case FlightCommandType::JOYSTICK_STEP:
        {
            if (!EnsureFlightControllerInited())
            {
                publish_ack("joystick", "error", "fc not inited");
                break;
            }
            if (!has_joystick_auth_.load())
            {
                publish_ack("joystick", "error", "no joystick authority");
                break;
            }
            T_DjiFlightControllerJoystickMode mode = {
                DJI_FLIGHT_CONTROLLER_HORIZONTAL_VELOCITY_CONTROL_MODE,
                DJI_FLIGHT_CONTROLLER_VERTICAL_VELOCITY_CONTROL_MODE,
                DJI_FLIGHT_CONTROLLER_YAW_ANGLE_RATE_CONTROL_MODE,
                DJI_FLIGHT_CONTROLLER_HORIZONTAL_BODY_COORDINATE,
                DJI_FLIGHT_CONTROLLER_STABLE_CONTROL_MODE_ENABLE,
            };
            DjiFlightController_SetJoystickMode(mode);
            T_DjiFlightControllerJoystickCommand jcmd = {0};
            jcmd.x = cmd.x * joystick_speed_m_s_;
            jcmd.y = cmd.y * joystick_speed_m_s_;
            jcmd.z = cmd.z * joystick_speed_m_s_;
            jcmd.yaw = (cmd.yaw > 0 ? yaw_angle_deg_ : (cmd.yaw < 0 ? -yaw_angle_deg_ : 0.0));
            if (DjiFlightController_ExecuteJoystickAction(jcmd) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
            {
                publish_ack("joystick", "error", "execute failed");
                break;
            }
            sleep_ms(cmd.hold_ms);
            T_DjiFlightControllerJoystickCommand stop = {0};
            DjiFlightController_ExecuteJoystickAction(stop);
            publish_ack("joystick", "ok", "done");
            break;
        }

        case FlightCommandType::JOYSTICK_HOLD_START:
            if (!EnsureFlightControllerInited())
            {
                publish_ack("joystick_hold_start", "error", "fc not inited");
            }
            else if (!has_joystick_auth_.load())
            {
                publish_ack("joystick_hold_start", "error", "no joystick authority");
            }
            else
            {
                StartJoystickHold(cmd);
                publish_ack("joystick_hold_start", "ok", "started");
            }
            break;

        case FlightCommandType::JOYSTICK_HOLD_STOP:
            StopJoystickHold();
            publish_ack("joystick_hold_stop", "ok", "stopped");
            break;

        case FlightCommandType::WAYPOINT_UPLOAD:
        {
            if (!EnsureFlightControllerInited())
            {
                publish_ack("waypoint_upload", "error", "fc not inited");
                break;
            }
            bool ok = UploadKmzFromUrl(cmd.waypoint_url, cmd.waypoint_local_path.empty() ? "/tmp/psdk_waypoint.kmz" : cmd.waypoint_local_path);
            if (ok)
            {
                cache_.put("current_flightId", cmd.flightid);
                if (telemetry_pub_)
                    telemetry_pub_->setWaypointFlightId(cmd.flightid);
                publish_ack("waypoint_upload", "ok", "uploaded");
            }
            else
            {
                publish_ack("waypoint_upload", "error", "upload failed");
            }
            break;
        }

        case FlightCommandType::WAYPOINT_START:
        {
            T_DjiReturnCode rc = DjiWaypointV3_Action(DJI_WAYPOINT_V3_ACTION_START);
            if (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
            {
                std::string flightid = cache_.get_or_default("current_flightId", "");
                if (telemetry_pub_ && !flightid.empty())
                    telemetry_pub_->setWaypointFlightId(flightid);
                publish_ack("waypoint_start", "ok", "started");
            }
            else
            {
                publish_ack("waypoint_start", "error", "start failed");
            }
            break;
        }

        case FlightCommandType::WAYPOINT_PAUSE:
        {
            T_DjiReturnCode rc = DjiWaypointV3_Action(DJI_WAYPOINT_V3_ACTION_PAUSE);
            if (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS || rc == 0x1000B)
            {
                publish_ack("waypoint_pause", "ok", "paused");
            }
            else
            {
                char buf[64];
                snprintf(buf, sizeof(buf), "pause failed (code: 0x%08llX)", rc);
                publish_ack("waypoint_pause", "error", buf);
            }
            break;
        }

        case FlightCommandType::WAYPOINT_RESUME:
            if (DjiWaypointV3_Action(DJI_WAYPOINT_V3_ACTION_RESUME) == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
                publish_ack("waypoint_resume", "ok", "resumed");
            else
                publish_ack("waypoint_resume", "error", "resume failed");
            break;

        case FlightCommandType::WAYPOINT_STOP:
            if (DjiWaypointV3_Action(DJI_WAYPOINT_V3_ACTION_STOP) == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
            {
                publish_ack("waypoint_stop", "ok", "stopped");
                cache_.put("current_flightId", "");
                if (telemetry_pub_)
                    telemetry_pub_->setWaypointFlightId("");
            }
            else
            {
                publish_ack("waypoint_stop", "error", "stop failed");
            }
            break;

        default:
            USER_LOG_WARN("Unhandled FlightCommandType");
            break;
        }
    }
}

// -------------------- 发布 ack（原版 JSON 格式）--------------------
void CommandExecutor::publish_ack(const std::string &cmd, const std::string &status, const std::string &msg)
{
    std::string flightid = cache_.get_or_default("current_flightId", "");
    json j;
    j["cmd"] = cmd;
    j["status"] = status;
    j["message"] = msg;
    j["flightId"] = flightid;

    std::string topic = "drone/" + drone_id_ + "/psdk/command/ack";
    mqtt_.publish(topic, j.dump());
}

// -------------------- Waypoint 回调发布（原版 JSON 格式）--------------------
void CommandExecutor::publish_waypoint_mission_state(T_DjiWaypointV3MissionState s)
{
    std::string flightid = cache_.get_or_default("current_flightId", "");
    json j;
    j["missionState"] = static_cast<int>(s.state);
    j["wayLineId"] = s.wayLineId;
    j["currentWaypointIndex"] = s.currentWaypointIndex;
    j["flightId"] = flightid;

    std::string topic = "drone/" + drone_id_ + "/psdk/waypoint/mission_state";
    mqtt_.publish(topic, j.dump());
}

void CommandExecutor::publish_waypoint_action_state(T_DjiWaypointV3ActionState s)
{
    std::string flightid = cache_.get_or_default("current_flightId", "");
    json j;
    j["actionState"] = static_cast<int>(s.state);
    j["wayLineId"] = s.wayLineId;
    j["currentWaypointIndex"] = s.currentWaypointIndex;
    j["actionGroupId"] = s.actionGroupId;
    j["actionId"] = s.actionId;
    j["flightId"] = flightid;

    std::string topic = "drone/" + drone_id_ + "/psdk/waypoint/action_state";
    mqtt_.publish(topic, j.dump());
}

void CommandExecutor::publish_display_mode(T_DjiFcSubscriptionDisplaymode mode)
{
    std::string flightid = cache_.get_or_default("current_flightId", "");
    json j;
    j["displayMode"] = static_cast<int>(mode);
    j["flightId"] = flightid;

    std::string topic = "drone/" + drone_id_ + "/psdk/telemetry/fcsub/display_mode";
    mqtt_.publish(topic, j.dump());
}

// -------------------- 其他 --------------------
void CommandExecutor::set_has_joystick_auth(bool has_auth)
{
    bool old = has_joystick_auth_.exchange(has_auth);
    if (old != has_auth && telemetry_pub_)
    {
        telemetry_pub_->setHasJoystickControl(has_auth);
        USER_LOG_INFO("Joystick control authority changed to %d", has_auth);
    }
}

std::string CommandExecutor::GetCurrentFlightId() const
{
    return cache_.get_or_default("current_flightId", "");
}
