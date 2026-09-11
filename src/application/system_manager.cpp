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
#include "h30t/h30t_rgb_stream_pipeline.hpp"
#include "local/video_file_source.hpp"
#include "yolo/h30t_yolo_service.hpp"
#include "local/localizer.hpp"
#include "local/minio_uploader.hpp"
#include "dji_logger.h"
#include "dji_platform.h"
#include "dji_liveview.h"
#include "dji_camera_manager.h"
#include "../common/osal/osal.h"
#include "dji_aircraft_info.h"
#include "3rdparty/json.hpp"
#include "app_config.hpp"

#include <cstdlib>
#include <iostream>
#include <chrono>
#include <opencv2/imgproc.hpp>

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
        USER_LOG_ERROR("H30T YOLO startup failed; RTSP will continue without detection");
    }

    // 1. 启动 H30T 视频流；RTSP 服务器由外部服务负责运行
    // 1. 创建 MQTT 桥接
    mqtt_ = std::unique_ptr<MqttBridge>(new MqttBridge(start_time_));
    const AppConfig &app = AppConfig::Instance();
    MqttConfig cfg;
    cfg.host=app.String("mqtt.host"); cfg.port=app.Int("mqtt.port"); cfg.keepalive=app.Int("mqtt.keepalive");
    cfg.username=app.String("mqtt.username"); cfg.password=app.String("mqtt.password");
    cfg.client_id=app.String("mqtt.client_id"); cfg.drone_id=app.String("mqtt.drone_id");

    drone_id_ = cfg.drone_id;

    if (!mqtt_->init(cfg))
    {
        USER_LOG_ERROR("[NODE][system_manager] MQTT init failed");
        return false;
    }

    // 2. 添加 MQTT 日志转发
    g_sys_mgr_for_log = this;
    T_DjiLoggerConsole mqttLogger = {0};
    mqttLogger.func = mqttLogCallback;
    mqttLogger.consoleLevel = DJI_LOGGER_CONSOLE_LOG_LEVEL_INFO;
    mqttLogger.isSupportColor = false;
    T_DjiReturnCode rc = DjiLogger_AddConsole(&mqttLogger);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
        USER_LOG_ERROR("[NODE][system_manager] MQTT logger attachment failed, rc=0x%08llX", rc);
    }
    else
    {
    }

    // 4. 启动 MQTT 依赖服务（遥测、FC 订阅等）
    if (!startMqttDependencies()) {
        USER_LOG_ERROR("[NODE][system_manager] MQTT dependencies startup failed");
        return false;
    }
    if (!startH30tStream()) {
        USER_LOG_ERROR("H30T stream startup failed; continuing in degraded mode");
    }
    if (cmd_exec_) cmd_exec_->SetH30tStreamController(h30t_stream_.get());
    running_ = true;
    return true;
}

// ============================================================================
// H30T 启动
// ============================================================================
bool SystemManager::startH30tStream()
{
    const AppConfig &app=AppConfig::Instance();   // 取全局配置单例，后续读配置项

    // 若配置 video.input_mode == "local_file"，走"本地视频文件"分支
    if(app.String("video.input_mode")=="local_file") {

        // 读取配置里指定的相机源字符串
        const std::string source=app.String("video.camera_source");
        uint8_t camera_source=0;   // 用于存放映射后的 PSDK 相机源枚举值

        // 将配置字符串映射为 PSDK 的相机源枚举
        if(source=="h30t_infrared")
            camera_source=static_cast<uint8_t>(DJI_CAMERA_MANAGER_SOURCE_IR_CAM);     // 红外相机
        else if(source=="h30t_zoom")
            camera_source=static_cast<uint8_t>(DJI_CAMERA_MANAGER_SOURCE_ZOOM_CAM);   // 变焦相机
        else if(source=="h30t_wide")
            camera_source=static_cast<uint8_t>(DJI_CAMERA_MANAGER_SOURCE_WIDE_CAM);   // 广角相机
        else {
            // 配置值不在支持列表内，报错并返回失败
            USER_LOG_ERROR("Invalid video.camera_source");
            return false;
        }

        // 初始化 PSDK Liveview（图传）模块，失败则报错返回
        if(DjiLiveview_Init()!=DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("DjiLiveview_Init failed for local video");
            return false;
        }
        local_liveview_initialized_=true;   // 标记 Liveview 已初始化，便于后续反初始化

        // 加载 H30T 的 RTSP 配置（URL、传输方式、缓冲上限）
        const H30tRtspConfig rtsp=h30t_config::LoadRtspConfig();

        // 组装视频管线配置：RTSP 地址、传输协议、最大队列字节数
        H30tStreamPipelineConfig output;
        output.rtsp_url=rtsp.rtsp_url;
        output.rtsp_transport=rtsp.transport;
        output.max_queue_bytes=rtsp.max_queue_bytes;

        // 创建 RGB 流管线对象，负责编码并把帧推到 RTSP
        local_video_pipeline_=std::unique_ptr<H30tRgbStreamPipeline>(new H30tRgbStreamPipeline());

        // 启动管线；失败则回滚（反初始化 Liveview、释放管线）并返回失败
        if(!local_video_pipeline_->Start(output)) {
            DjiLiveview_Deinit();
            local_liveview_initialized_=false;
            local_video_pipeline_.reset();
            return false;
        }

        // 创建本地视频文件源对象，负责从文件逐帧读取
        video_input_=std::unique_ptr<VideoFileSource>(new VideoFileSource());
        std::string error;   // 用于接收启动失败时的错误信息

        // 启动视频文件源：
        //   - video.file_path：视频文件路径
        //   - camera_source：相机源
        //   - video.loop=="true"：是否循环播放
        //   - 帧回调：每读到一帧就交给 YOLO 检测（若 yolo_ 存在）
        // 失败则回滚所有已启动组件并返回失败
        if(!video_input_->Start(app.String("video.file_path"),
                                camera_source,
                                app.String("video.loop")=="true",
                                [this](const uint8_t*d,uint32_t n,uint16_t w,uint16_t h,
                                    const H30tFrameMetadata&m){
                                    if(yolo_) yolo_->SubmitRgbFrame(d,n,w,h,m);   // 提交 RGB 帧给 YOLO
                                },
                                error)) 
        {
            USER_LOG_ERROR("Local video start failed: %s",error.c_str());
            local_video_pipeline_->Stop();
            local_video_pipeline_.reset();
            DjiLiveview_Deinit();
            local_liveview_initialized_=false;
            return false;
        }

        // 成功：本地视频输入已启动，H30T 实时图传被禁用
        USER_LOG_INFO("Local video input started; H30T image stream is disabled");
        return true;
    }

    // ========== 分支二：H30T 实时流模式 ==========
    // 走到这里说明 video.input_mode != "local_file"，使用 H30T 实时图传

    // 1. 确定 H30T 挂载位置（如挂载点 3，来自配置 h30t.mount）
    h30t_mount_ = AppConfig::Instance().Int("h30t.mount");

    // 2. 启动 H30T 流控制器
    h30t_stream_ = std::unique_ptr<H30tStreamController>(
        new H30tStreamController(
            [this](const std::string &cmd, const std::string &status,
                   const std::string &message) {
                // H30T 状态回调：把命令执行状态通过 MQTT 发 ACK 给地面端
                if (!mqtt_) return;                      // MQTT 未就绪则不发
                json ack;
                ack["cmd"] = cmd;                        // 命令名
                ack["status"] = status;                  // 执行状态
                ack["message"] = message;                // 附加信息
                ack["flightId"] = cmd_exec_ ? cmd_exec_->GetCurrentFlightId() : "";  // 当前架次号
                mqtt_->publish("drone/" + drone_id_ + "/psdk/command/ack", ack.dump());
            }));

    // 1. 拷贝 YOLO 服务的 shared_ptr，供回调捕获（避免 this 失效）
    const std::shared_ptr<H30tYoloService> yolo = yolo_;

    // 2. 设置 RGB 帧回调：每收到一帧就提交给 YOLO 检测
    h30t_stream_->SetRgbFrameCallback(
        [yolo](const uint8_t *data, uint32_t length, uint16_t width, uint16_t height,
               const H30tFrameMetadata &metadata) {
            if (yolo) yolo->SubmitRgbFrame(data, length, width, height, metadata);
        });

    // 启动流控制器的工作线程（负责拉流、解码、分发帧）
    if (!h30t_stream_->StartWorker()) {
        USER_LOG_ERROR("H30T worker thread start failed");
        h30t_stream_.reset();
        return false;
    }

    // 请求开始 H30T 流（在指定挂载点上）
    if (!h30t_stream_->RequestStart(h30t_mount_)) {
        h30t_stream_->Shutdown();
        h30t_stream_.reset();
        return false;
    }
    return true;
}

bool SystemManager::startYolo()
{
    // 1. 创建 YOLO 服务实例（智能指针管理）
    yolo_ = std::shared_ptr<H30tYoloService>(new H30tYoloService());
    std::string error;
    // 2. 启动 YOLO 服务，传入 Lambda 回调
    if (!yolo_->Start([this](const std::string &payload, const std::vector<uint8_t> &jpeg) {
            // 3. YOLO 回调：将检测结果通过 MQTT 发布
            if (mqtt_) {
                mqtt_->publish("drone/" + drone_id_ + "/psdk/h30t/detections",
                               payload, 1, false);
                std::string locations;
                if (!localizer_ || !localizer_->processDetectionJson(payload, locations)) return;
                if (!minio_uploader_) return;
                const json input = json::parse(payload);
                const bool infrared = input.value("source", std::string()) == "h30t_infrared";
                const std::string name = std::to_string(input.value("frameId", uint64_t(0))) + ".jpg";
                minio_uploader_->Enqueue(jpeg, name, infrared, locations,
                    [this](const MinioUploadResult &result, const std::string &locationJson) {
                        if (!mqtt_) return;
                        json feedback;
                        feedback["flightId"] = cmd_exec_ ? cmd_exec_->GetCurrentFlightId() : "";
                        feedback["code"] = result.success ? 200 : -1;
                        feedback["Msg"] = result.success ? "Success,图片已成功上传至MinIO" : ("图片上传失败：" + result.error);
                        feedback["fileUrl"] = result.success ? result.file_url : "";
                        mqtt_->publish("drone/" + drone_id_ + "/psdk/h30t/upload", feedback.dump(), 1, false);
                        if (result.success) {
                            json locations = json::parse(locationJson);
                            for (auto &item : locations["locations"]) { item["imageUrl"] = result.file_url; item["objectName"] = result.object_name; }
                            mqtt_->publish("drone/" + drone_id_ + "/psdk/h30t/locations", locations.dump(), 1, false);
                        }
                    });
            }
        }, [this](const uint8_t *data, uint32_t length, uint16_t width, uint16_t height) {
            if (h30t_stream_) h30t_stream_->PushProcessedRgb(data, length, width, height);
            if (local_video_pipeline_) { cv::Mat bgr(height,width,CV_8UC3,const_cast<uint8_t*>(data)); cv::Mat rgb; cv::cvtColor(bgr,rgb,cv::COLOR_BGR2RGB); local_video_pipeline_->PushRgb(rgb.data,static_cast<uint32_t>(rgb.total()*rgb.elemSize()),width,height); }
        }, error)) {
        // 4. 启动失败，输出错误信息
        USER_LOG_ERROR("H30T YOLO disabled: %s", error.c_str());
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
// MQTT 依赖启动
// ============================================================================
bool SystemManager::startMqttDependencies()
{
    const MqttConfig &cfg = mqtt_->cfg();

    // 1. 创建遥测发布器（5Hz）
    telemetry_ = std::unique_ptr<TelemetryPublisher>(new TelemetryPublisher(*mqtt_));
    telemetry_->start_5hz();

    // 2. 创建位置遥测发布器（50Hz）
    telemetry_pos_ = std::unique_ptr<TelemetryPosPublisher>(
        new TelemetryPosPublisher(*mqtt_));
    localizer_ = std::unique_ptr<Localizer>(new Localizer(cfg.drone_id));
    minio_uploader_ = std::unique_ptr<MinioUploader>(new MinioUploader());
    std::string minio_error;
    if (!minio_uploader_->Start(minio_error)) {
        USER_LOG_ERROR("MinIO uploader disabled: %s", minio_error.c_str());
        minio_uploader_.reset();
    }
    telemetry_pos_->setLocalizer(localizer_.get());
    telemetry_pos_->start();

    // 3. 创建命令执行器
    cmd_exec_ = std::unique_ptr<CommandExecutor>(new CommandExecutor(*mqtt_, cfg.drone_id));
    cmd_exec_->SetTelemetryPublisher(telemetry_.get());
    if (!cmd_exec_->EnsureFlightControllerInited()) {
        USER_LOG_ERROR("Flight controller initialization failed; local RTSP remains available");
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

    // 6. 创建降落检测器
    landing_detector_ = std::unique_ptr<LandingDetector>(new LandingDetector(
        *mqtt_, cfg.drone_id,
        [this]() { return cmd_exec_ ? cmd_exec_->GetCurrentFlightId() : ""; }));

    fc_mgr_->addObserver(telemetry_.get());
    fc_mgr_->addObserver(telemetry_pos_.get());
    fc_mgr_->addObserver(landing_detector_.get());

    // 7. 启动 FC 订阅
    if (!fc_mgr_->start()) {
        USER_LOG_ERROR("FcSubscriptionManager start failed");
        return false;
    }
    USER_LOG_INFO("[NODE][system_manager] MQTT/telemetry/FC services started");

    return true;
}

// ============================================================================
// H30T 停止
// ============================================================================
void SystemManager::stopH30tStream()
{
    if(video_input_) { video_input_->Stop(); video_input_.reset(); }
    if(local_video_pipeline_) { local_video_pipeline_->Stop(); local_video_pipeline_.reset(); }
    if(local_liveview_initialized_) { DjiLiveview_Deinit(); local_liveview_initialized_=false; }
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
    localizer_.reset();
    if (minio_uploader_) { minio_uploader_->Stop(); minio_uploader_.reset(); }

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

    // 3. 清理命令执行器
    if (cmd_exec_) {
        cmd_exec_.reset();
        USER_LOG_INFO("CommandExecutor stopped");
    }

    // 4. 断开 MQTT
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
