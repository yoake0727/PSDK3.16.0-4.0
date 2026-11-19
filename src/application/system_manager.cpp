/**
 * @file system_manager.cpp
 * @brief 系统管理器实现
 */

#include "system_manager.hpp"
#include "mqtt/mqtt_bridge.hpp"
#include "mqtt/telemetry_publisher.hpp"
#include "mqtt/telemetry_pos_publisher.hpp"
#include "flight_command/command_executor.hpp"
#include "subscription/fc_subscription_manager.hpp"
#include "subscription/landing_detector.hpp"
#include "h30t/h30t_stream_controller.hpp"
#include "h30t/h30t_config.hpp"
<<<<<<< HEAD
#include "downloader/h30t_downloader.hpp"
#include "downloader/download_config.hpp"
#include "widget/h30t_widget.hpp"
=======
#include "yolo/h30t_yolo_service.hpp"
>>>>>>> 56d8b0fd8be8e9ee8398903895292230f38a96df
#include "dji_logger.h"
#include "dji_platform.h"
#include "../common/osal/osal.h"
#include "dji_aircraft_info.h"
#include "3rdparty/json.hpp"

#include <cstdlib>
#include <iostream>
#include <chrono>

using json = nlohmann::json;

static SystemManager *g_sys_mgr_for_log = nullptr;

SystemManager::SystemManager()
    : start_time_(std::chrono::steady_clock::now())
{
}

SystemManager::~SystemManager()
{
    shutdown();
}

T_DjiReturnCode SystemManager::mqttLogCallback(const uint8_t *data, uint16_t dataLen)
{
    if (!g_sys_mgr_for_log || !g_sys_mgr_for_log->mqtt_ || !data || dataLen == 0)
        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;

    std::string raw((const char *)data, dataLen);
    while (!raw.empty() && (raw.back() == '\n' || raw.back() == '\r'))
        raw.pop_back();

    nlohmann::json j;
    j["message"] = raw;
    std::string payload = j.dump();

    std::string topic = "drone/" + g_sys_mgr_for_log->drone_id_ + "/psdk/log";
    g_sys_mgr_for_log->mqtt_->publish(topic, payload, 1, false);

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

bool SystemManager::init()
{
    running_ = true;
    if (!startYolo()) {
        USER_LOG_WARN("H30T YOLO startup failed; RTSP will continue without detection");
    }
    // USER_LOG_INFO("[NODE][system_manager] init begin");

    // 0. 自定义控件 UI 配置必须先于相机管理器初始化
    //    PSDK 下载模块初始化时会校验控件 UI 配置，未注册会导致下载数据初始化失败(0x000000E3)
    const char *widget_enable_env = std::getenv("H30T_WIDGET_ENABLE");
    const char *download_enable_env = std::getenv("H30T_DL_ENABLE");
    const bool widget_config_required =
        (widget_enable_env == nullptr || std::string(widget_enable_env) != "0")
        || (download_enable_env != nullptr && std::string(download_enable_env) == "1");
    if (widget_config_required) {
        const char *widget_dir_env = std::getenv("H30T_WIDGET_DIR");
        std::string widget_error;
        if (!H30tWidget::InitUiConfig(widget_dir_env != nullptr ? widget_dir_env : "",
                                      &widget_error)) {
            USER_LOG_ERROR("[NODE][system_manager] widget ui config init failed: %s",
                           widget_error.c_str());
        }
    }

    // 1. 启动 H30T 视频流；RTSP 服务器由外部服务负责运行
    if (!startH30tStream()) {
        USER_LOG_WARN("H30T stream startup failed; continuing in degraded mode");
        // 不返回 false，允许部分功能运行
    }
    // USER_LOG_INFO("[NODE][system_manager] H30T stream stage completed");

    // 1. 创建 MQTT 桥接
    mqtt_ = std::unique_ptr<MqttBridge>(new MqttBridge(start_time_));
    // USER_LOG_INFO("[NODE][system_manager] MQTT bridge constructed");
    MqttConfig cfg;

    // 环境变量覆盖（省略，与你原来的相同）
    const char *host = std::getenv("MQTT_HOST");
    if (host) cfg.host = host;
    const char *port = std::getenv("MQTT_PORT");
    if (port) cfg.port = std::atoi(port);
    const char *user = std::getenv("MQTT_USER");
    if (user) cfg.username = user;
    const char *pass = std::getenv("MQTT_PASS");
    if (pass) cfg.password = pass;
    const char *client = std::getenv("MQTT_CLIENT_ID");
    if (client) cfg.client_id = client;
    const char *drone = std::getenv("DRONE_ID");
    if (drone) cfg.drone_id = drone;

    drone_id_ = cfg.drone_id;
    // USER_LOG_INFO("[NODE][system_manager] MQTT configuration loaded from defaults/environment");

    if (!mqtt_->init(cfg))
    {
        USER_LOG_ERROR("[NODE][system_manager] MQTT init failed");
        return false;
    }
    // USER_LOG_INFO("[NODE][system_manager] MQTT initialized");

    // 2. 添加 MQTT 日志转发
    g_sys_mgr_for_log = this;
    T_DjiLoggerConsole mqttLogger = {0};
    mqttLogger.func = mqttLogCallback;
    mqttLogger.consoleLevel = DJI_LOGGER_CONSOLE_LOG_LEVEL_INFO;
    mqttLogger.isSupportColor = false;
    T_DjiReturnCode rc = DjiLogger_AddConsole(&mqttLogger);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
        USER_LOG_WARN("[NODE][system_manager] MQTT logger attachment failed, rc=0x%08llX", rc);
    }
    else
    {
        USER_LOG_INFO("[NODE][system_manager] MQTT logger attached");
    }

    // 4. 启动 MQTT 依赖服务（遥测、FC 订阅等）
    if (!startMqttDependencies()) {
        USER_LOG_ERROR("[NODE][system_manager] MQTT dependencies startup failed");
        return false;
    }
    // USER_LOG_INFO("[NODE][system_manager] MQTT dependencies completed");

    running_ = true;
    // USER_LOG_INFO("[NODE][system_manager] init completed successfully");
    return true;
}

// ============================================================================
// H30T 启动
// ============================================================================
bool SystemManager::startH30tStream()
{
    // USER_LOG_INFO("[NODE][system_manager] H30T stream start begin; external RTSP server is required");
    // 1. 确定 H30T 挂载位置
    h30t_mount_ = 2;  // 默认挂载点 1
    const char *mount_env = std::getenv("H30T_MOUNT");
    if (mount_env) {
        int mount = std::atoi(mount_env);
        if (mount >= 1 && mount <= 3) {
            h30t_mount_ = mount;
        }
    }

    // 2. 启动 H30T 流控制器
    h30t_stream_ = std::unique_ptr<H30tStreamController>(
        new H30tStreamController(
            [this](const std::string &cmd, const std::string &status,
                   const std::string &message) {
                // H30T 状态回调：通过 MQTT 发送 ACK
                if (!mqtt_) return;
                json ack;
                ack["cmd"] = cmd;
                ack["status"] = status;
                ack["message"] = message;
                ack["flightId"] = cmd_exec_ ? cmd_exec_->GetCurrentFlightId() : "";
                mqtt_->publish("drone/" + drone_id_ + "/psdk/command/ack", ack.dump());
            }));
    const std::shared_ptr<H30tYoloService> yolo = yolo_;
    h30t_stream_->SetRgbFrameCallback(
        [yolo](const uint8_t *data, uint32_t length, uint16_t width, uint16_t height) {
            if (yolo) yolo->SubmitRgbFrame(data, length, width, height);
        });
    // USER_LOG_INFO("[NODE][system_manager] H30T stream controller constructed");

    if (!h30t_stream_->StartWorker()) {
        USER_LOG_ERROR("H30T worker thread start failed");
        h30t_stream_.reset();
        return false;
    }
    // USER_LOG_INFO("[NODE][system_manager] H30T worker started");

    if (!h30t_stream_->RequestStart(h30t_mount_)) {
        USER_LOG_ERROR("H30T dual stream start request failed on mount %d", h30t_mount_);
        h30t_stream_->Shutdown();
        h30t_stream_.reset();
        return false;
    }

    USER_LOG_INFO("[NODE][system_manager] H30T dual stream start requested on mount %d", h30t_mount_);
    return true;
}

bool SystemManager::startYolo()
{
    yolo_ = std::shared_ptr<H30tYoloService>(new H30tYoloService());
    std::string error;
    if (!yolo_->Start([this](const std::string &payload) {
            if (mqtt_) {
                mqtt_->publish("drone/" + drone_id_ + "/psdk/h30t/detections",
                               payload, 1, false);
            }
        }, error)) {
        USER_LOG_WARN("H30T YOLO disabled: %s", error.c_str());
        yolo_.reset();
        return false;
    }
    USER_LOG_INFO("H30T YOLO inference worker started");
    return true;
}

void SystemManager::stopYolo()
{
    if (!yolo_) return;
    yolo_->Stop();
    yolo_.reset();
    USER_LOG_INFO("H30T YOLO inference worker stopped");
}

// ============================================================================
// H30T 原图下载服务
// ============================================================================
bool SystemManager::startH30tDownloader()
{
    const char *enable = std::getenv("H30T_DL_ENABLE");
    if (enable == nullptr || std::string(enable) != "1") {
        USER_LOG_INFO("[NODE][system_manager] H30T downloader disabled (H30T_DL_ENABLE != 1)");
        return true;
    }

    if (!H30tWidget::UiConfigReady()) {
        USER_LOG_ERROR("[NODE][system_manager] widget ui config is not ready; "
                       "camera media download will fail with 0x000000E3");
    }

    H30tDownloadConfig config = h30t_download_config::LoadFromEnvironment();
    if (std::getenv("H30T_DL_MOUNT") == nullptr) {
        config.mount = h30t_mount_;  // 默认与视频流使用同一挂载点
    }

    h30t_downloader_ = std::unique_ptr<H30tDownloader>(new H30tDownloader(
        [this](const std::string &cmd, const std::string &status,
               const std::string &message) {
            if (!mqtt_) return;
            json ack;
            ack["cmd"] = cmd;
            ack["status"] = status;
            ack["message"] = message;
            ack["flightId"] = cmd_exec_ ? cmd_exec_->GetCurrentFlightId() : "";
            mqtt_->publish("drone/" + drone_id_ + "/psdk/command/ack", ack.dump());
        }));

    std::string error;
    if (!h30t_downloader_->Start(config, &error)) {
        USER_LOG_ERROR("[NODE][system_manager] H30T downloader start failed: %s", error.c_str());
        h30t_downloader_.reset();
        return false;
    }

    if (cmd_exec_) {
        cmd_exec_->SetH30tDownloader(h30t_downloader_.get());
    }

    // 自定义控件：把下载参数暴露到 Pilot 2（H30T_WIDGET_ENABLE=0 可关闭）
    const char *widget_enable = std::getenv("H30T_WIDGET_ENABLE");
    if (widget_enable == nullptr || std::string(widget_enable) != "0") {
        const char *widget_dir = std::getenv("H30T_WIDGET_DIR");
        h30t_widget_ = std::unique_ptr<H30tWidget>(new H30tWidget(
            h30t_downloader_.get(), widget_dir != nullptr ? widget_dir : ""));
        std::string widget_error;
        if (!h30t_widget_->Start(&widget_error)) {
            USER_LOG_WARN("[NODE][system_manager] H30T widget start failed: %s",
                          widget_error.c_str());
            h30t_widget_.reset();
        }
    }

    USER_LOG_INFO("[NODE][system_manager] H30T downloader started, root=%s, incoming=%s",
                  config.root_dir.c_str(), config.incoming_dir.c_str());
    return true;
}

void SystemManager::stopH30tDownloader()
{
    if (!h30t_downloader_) return;
    if (h30t_widget_) {
        h30t_widget_->Stop();
        h30t_widget_.reset();
    }
    if (cmd_exec_) {
        cmd_exec_->SetH30tDownloader(nullptr);
    }
    h30t_downloader_->Shutdown();
    h30t_downloader_.reset();
    USER_LOG_INFO("H30T downloader stopped");
}

// ============================================================================
// MQTT 依赖启动
// ============================================================================
bool SystemManager::startMqttDependencies()
{
    // USER_LOG_INFO("[NODE][system_manager] MQTT dependencies start begin");
    // if (!mqtt_ || !mqtt_->is_connected()) {
    //     USER_LOG_ERROR("MQTT not available");
    //     return false;
    // }

    const MqttConfig &cfg = mqtt_->cfg();

    // 1. 创建遥测发布器（5Hz）
    telemetry_ = std::unique_ptr<TelemetryPublisher>(new TelemetryPublisher(*mqtt_));
    telemetry_->start_5hz();
    USER_LOG_INFO("TelemetryPublisher started (5Hz)");

    // 2. 创建位置遥测发布器（50Hz）
    telemetry_pos_ = std::unique_ptr<TelemetryPosPublisher>(
        new TelemetryPosPublisher(*mqtt_));
    telemetry_pos_->start();
    USER_LOG_INFO("TelemetryPosPublisher started (50Hz)");

    // 3. 创建命令执行器
    cmd_exec_ = std::unique_ptr<CommandExecutor>(new CommandExecutor(*mqtt_, cfg.drone_id));
    cmd_exec_->SetTelemetryPublisher(telemetry_.get());
    cmd_exec_->SetH30tStreamController(h30t_stream_.get());

    // 3.1 可选：启动 H30T 原图下载服务（H30T_DL_ENABLE=1）
    if (!startH30tDownloader()) {
        USER_LOG_WARN("H30T downloader startup failed; download commands are disabled");
    }

    if (!cmd_exec_->EnsureFlightControllerInited()) {
        USER_LOG_WARN("Flight controller initialization failed; local RTSP remains available");
    }
    Osal_TaskSleepMs(3000);
    USER_LOG_INFO("[NODE][system_manager] flight controller stabilization wait completed");

    // 4. 设置 MQTT 命令回调
    mqtt_->set_message_callback([this](const std::string &topic,
                                        const std::string &payload,
                                        bool retained) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
            now - start_time_).count();
        if (retained || elapsed < 5) {
            USER_LOG_INFO("Ignoring retained/early command: %s", topic.c_str());
            return;
        }
        if (cmd_exec_) {
            cmd_exec_->HandleMqttCommand(topic, payload);
        }
    });

    std::string ctl_topic = "drone/" + cfg.drone_id + "/psdk/command/ctl";
    mqtt_->subscribe(ctl_topic, 1);
    USER_LOG_INFO("Subscribed to %s", ctl_topic.c_str());

    // 5. 创建 FC 订阅管理器
    fc_mgr_ = std::unique_ptr<FcSubscriptionManager>(new FcSubscriptionManager());
    USER_LOG_INFO("[NODE][system_manager] FC subscription manager constructed");

    // 6. 创建降落检测器
    landing_detector_ = std::unique_ptr<LandingDetector>(new LandingDetector(
        *mqtt_, cfg.drone_id,
        [this]() { return cmd_exec_ ? cmd_exec_->GetCurrentFlightId() : ""; }));
    USER_LOG_INFO("[NODE][system_manager] landing detector constructed");

    fc_mgr_->addObserver(telemetry_.get());
    fc_mgr_->addObserver(telemetry_pos_.get());
    fc_mgr_->addObserver(landing_detector_.get());

    // 7. 启动 FC 订阅
    if (!fc_mgr_->start()) {
        USER_LOG_ERROR("FcSubscriptionManager start failed");
        return false;
    }
    USER_LOG_INFO("FC subscription manager started");

    return true;
}

// ============================================================================
// H30T 停止
// ============================================================================
void SystemManager::stopH30tStream()
{
    if (h30t_stream_) {
        h30t_stream_->Shutdown();
        h30t_stream_.reset();
        USER_LOG_INFO("H30T stream stopped");
    }

}

// ============================================================================
// MQTT 依赖停止
// ============================================================================
void SystemManager::stopMqttDependencies()
{
    // 1. 先停止发布器（不再接收新数据）
    if (telemetry_pos_) {
        telemetry_pos_->stop();
        telemetry_pos_.reset();
        USER_LOG_INFO("TelemetryPosPublisher stopped");
    }

    if (telemetry_) {
        telemetry_->stop();
        telemetry_.reset();
        USER_LOG_INFO("TelemetryPublisher stopped");
    }

    // 2. 停止 FC 订阅管理器
    if (fc_mgr_) {
        fc_mgr_->stop();
        fc_mgr_.reset();
        USER_LOG_INFO("FC subscription manager stopped");
    }

    // 3. 停止 H30T 原图下载服务（必须在 MQTT 关闭前，Ack 依赖 mqtt_）
    stopH30tDownloader();

    // 4. 清理命令执行器
    if (cmd_exec_) {
        cmd_exec_.reset();
        USER_LOG_INFO("CommandExecutor stopped");
    }

    // 5. 断开 MQTT
    if (mqtt_) {
        mqtt_->close();
        mqtt_.reset();
        USER_LOG_INFO("MQTT disconnected");
    }

    g_sys_mgr_for_log = nullptr;
}

void SystemManager::shutdown()
{
    if (!running_.exchange(false))
        return;
    // 按依赖顺序逆向停止
    stopH30tStream();
    stopYolo();
    stopMqttDependencies();
    g_sys_mgr_for_log = nullptr;
    USER_LOG_INFO("[NODE][system_manager] shutdown complete");
}
