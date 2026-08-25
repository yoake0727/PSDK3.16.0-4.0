/**
 * @file command_executor.h
 * @brief 命令执行器：解析 MQTT 命令，在独立线程中执行 PSDK 飞行控制 API
 */

#pragma once

#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <queue>

#include "dji_typedef.h"
#include "dji_flight_controller.h"
#include "dji_waypoint_v3.h"
#include "dji_fc_subscription.h"
#include "mqtt/mqtt_bridge.hpp"
#include "simple_cache.h"
#include "gimbal/gimbal_controller.hpp"

// 前向声明
class TelemetryPublisher;
class H30tStreamController; 

// 命令类型枚举（与原版一致）
enum class FlightCommandType
{
    TAKEOFF,
    LAND,
    FORCE_LAND,
    CANCEL_LAND,
    GO_HOME,
    CANCEL_GO_HOME,
    MOTOR_ON,
    MOTOR_OFF,
    EMERGENCY_STOP,
    JOYSTICK_STEP,
    OBTAIN_JOYSTICK_AUTH,
    RELEASE_JOYSTICK_AUTH,
    WAYPOINT_UPLOAD,
    WAYPOINT_START,
    WAYPOINT_PAUSE,
    WAYPOINT_RESUME,
    WAYPOINT_STOP,
    CONFIRM_LANDING,
    JOYSTICK_HOLD_START,
    JOYSTICK_HOLD_STOP,
    GIMBAL_ROTATE
};

// 命令结构体（与原版相同）
struct FlightCommand
{
    FlightCommandType type;
    double x = 0.0, y = 0.0, z = 0.0;
    double yaw = 0.0;
    uint32_t hold_ms = 1000;
    std::string waypoint_url;
    std::string waypoint_local_path;
    std::string reason;
    std::string flightid = "";
    E_DjiMountPosition gimbal_mount = DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1;
    E_DjiGimbalRotationMode gimbal_mode = DJI_GIMBAL_ROTATION_MODE_RELATIVE_ANGLE;
    float gimbal_pitch = 0.0F, gimbal_roll = 0.0F, gimbal_yaw = 0.0F;
    double gimbal_time = 1.0;
};

class CommandExecutor
{
public:
    CommandExecutor(MqttBridge &mqtt, const std::string &drone_id);
    ~CommandExecutor();

    // 设置遥测发布器（用于更新 flightId 和控制权状态）
    void SetTelemetryPublisher(TelemetryPublisher *tp);
    // 新增：设置 H30T 流控制器（用于 H30T 相关命令）
    void SetH30tStreamController(H30tStreamController *controller);

    // 外部调用入口：解析 MQTT 命令并入队（在 MQTT 回调线程中调用）
    void HandleMqttCommand(const std::string &topic, const std::string &payload);

    // 供 C 风格回调调用的发布方法（与原版同名）
    void publish_waypoint_mission_state(T_DjiWaypointV3MissionState s);
    void publish_waypoint_action_state(T_DjiWaypointV3ActionState s);
    void publish_display_mode(T_DjiFcSubscriptionDisplaymode s);

    // 获取当前飞行任务 ID
    std::string GetCurrentFlightId() const;

    // 控制权状态访问（供外部查询，也可由 ControlDevice 回调直接设置）
    void set_has_joystick_auth(bool has_auth);
    bool HasJoystickControl() const { return has_joystick_auth_.load(); }
        // 内部初始化
    bool EnsureFlightControllerInited();

private:

    // ⭐ 新增：H30T 相关命令处理
    void HandleH30tCommand(const std::string &cmd, const std::string &payload);
    
    bool AcquireJoystickAuthority();
    bool UploadKmzFromUrl(const std::string &url, const std::string &local_path);

    // 工作线程函数
    void FlightWorkerThread();

    // 发布 ack（保持原版 JSON 格式）
    void publish_ack(const std::string &cmd, const std::string &status, const std::string &msg);

    // 入队命令（线程安全）
    void EnqueueFlightCommand(const FlightCommand &cmd);

    // 摇杆持续控制相关
    void StartJoystickHold(const FlightCommand &cmd);
    void StopJoystickHold();

private:
    MqttBridge &mqtt_;
    std::string drone_id_;
    TelemetryPublisher *telemetry_pub_ = nullptr;

    // ⭐ 新增：H30T 流控制器（原始指针，生命周期由 SystemManager 管理）
    H30tStreamController *h30t_stream_controller_ = nullptr;
    GimbalController gimbal_controller_;
    // 工作线程与队列
    std::thread flight_worker_;
    std::atomic<bool> flight_worker_run_{false};
    std::queue<FlightCommand> flight_cmd_queue_;
    std::mutex queue_mtx_;
    std::condition_variable queue_cv_;

    // 状态标志
    std::atomic<bool> fc_inited_{false};
    std::atomic<bool> has_joystick_auth_{false};
    SimpleCache cache_; // 存储 current_flightId

    // 摇杆持续控制线程
    std::thread joystick_hold_thread_;
    std::atomic<bool> joystick_hold_running_{false};
    std::mutex joystick_hold_mtx_;
    FlightCommand joystick_hold_cmd_;

    // 参数（与原版相同）
    const double joystick_speed_m_s_ = 2.0;
    const double yaw_angle_deg_ = 30.0;
};
