#include "h30t_yolo_service.hpp"

#include "3rdparty/json.hpp"
#include "dji_logger.h"
#include "app_config.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <sstream>

using json = nlohmann::json;


/**
 * 构造：初始化运行状态和帧标志。
 * 此时不加载模型、不启动线程，等 Start() 才做。
 */
H30tYoloService::H30tYoloService() : running_(false), has_frame_(false) {}

/**
 * 析构：自动调用 Stop()，保证工作线程退出、回调清空。
 */
H30tYoloService::~H30tYoloService() { Stop(); }

/**
 * 从项目统一 YAML 读取 YOLO 模型、标签和推理参数。
 */
YoloDetectorConfig H30tYoloService::LoadConfig()
{
    YoloDetectorConfig config;
    const AppConfig &app=AppConfig::Instance();
    config.model_path=app.String("yolo.model"); config.labels_path=app.String("yolo.labels");
    config.input_size=app.Int("yolo.input_size"); config.confidence_threshold=static_cast<float>(app.Double("yolo.confidence")); config.nms_threshold=static_cast<float>(app.Double("yolo.nms"));
    return config;
}

/**
 * 启动服务：加载模型 + 启动工作线程。
 *
 * 参数：
 *   callback       : 检测结果回调，参数 (JSON 字符串, JPEG 字节流)
 *                    用于 MQTT 发布检测结果和 MinIO 上传标注图
 *   frame_callback : 标注帧回调，参数 (data, len, w, h)
 *                    用于 RTSP 推流
 *   error          : 输出，启动失败时的错误信息
 *
 * 返回：
 *   true  : 模型加载成功，线程已启动
 *   false : 模型加载失败，error 里有原因
 *
 * 注意：先调 Stop() 保证重复 Start 不会泄漏线程。
 */
bool H30tYoloService::Start(const YoloResultCallback &callback,
                            const YoloAnnotatedFrameCallback &frame_callback,
                            std::string &error)
{
    Stop();  // 清理旧状态

    const YoloDetectorConfig config = LoadConfig();
    USER_LOG_INFO("H30T YOLO loading model: %s; labels: %s",
                  config.model_path.c_str(), config.labels_path.c_str());

    if (!detector_.Load(config, error)) return false;  // 加载模型

    callback_ = callback;
    frame_callback_ = frame_callback;
    running_ = true;
    worker_ = std::thread(&H30tYoloService::WorkerLoop, this);  // 启动工作线程
    return true;
}

/**
 * 上游推帧入口（由 DJI Liveview 回调线程调用）。
 *
 * 流程：
 *   1. 入参校验（空指针、宽高为 0、长度不足）
 *   2. 加锁
 *   3. clone 出独立可写的 Mat（data 是上游缓冲，回调返回后可能失效）
 *   4. 保存 metadata（时间戳、frameId、相机源）
 *   5. 置 has_frame_ 并唤醒工作线程
 *
 * 设计：
 *   - 只保留最新帧：pending_frame_ 被覆盖，旧帧丢弃，避免延迟累积
 *   - 注意 clone 在锁内，大帧会短暂阻塞上游回调
 */
void H30tYoloService::SubmitRgbFrame(const uint8_t *data, uint32_t length,
                                     uint16_t width, uint16_t height,
                                     const H30tFrameMetadata &metadata)
{
    // 入参校验：空指针、宽高为 0、长度不足 w*h*3 都拒绝
    if (!data || width == 0 || height == 0 ||
        length < static_cast<uint32_t>(width) * height * 3U) return;

    std::lock_guard<std::mutex> lock(mutex_);
    if (!running_) return;

    // 用上游缓冲构造 Mat 视图，再 clone 成独立可写副本
    cv::Mat view(height, width, CV_8UC3, const_cast<uint8_t *>(data));
    pending_frame_ = view.clone();
    pending_metadata_ = metadata;
    has_frame_ = true;
    condition_.notify_one();  // 唤醒工作线程
}

/**
 * 工作线程主循环。
 *
 * 每轮：
 *   1. 等条件（running_ 为 false 或 has_frame_ 为 true）
 *   2. 停止则退出
 *   3. 取出待处理帧和 metadata，清 has_frame_（锁内）
 *   4. 锁外推理
 *   5. 推理失败：推原始帧，continue
 *   6. 构造 JSON（时间戳、source、尺寸、detections）
 *   7. 逐个检测：填 JSON、画框、画标签
 *   8. 编码 JPEG，调 callback_（MQTT/上传）和 frame_callback_（推流）
 */
void H30tYoloService::WorkerLoop()
{
    while (true) {
        cv::Mat frame;
        H30tFrameMetadata metadata = {};
        {
            // 1. 等待新帧或停止信号
            std::unique_lock<std::mutex> lock(mutex_);
            condition_.wait(lock, [this]() { return !running_ || has_frame_; });

            // 2. 停止则退出
            if (!running_) break;

            // 3. 取出待处理帧（锁外推理，避免阻塞 SubmitRgbFrame）
            frame = pending_frame_;
            metadata = pending_metadata_;
            has_frame_ = false;
        }

        const std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

        // 4. 推理
        std::string error;
        const std::vector<YoloDetection> detections = detector_.Detect(frame, error);

        // 5. 推理失败：推原始帧给推流，跳过 JSON 构造
        if (!error.empty()) {
            USER_LOG_WARN("H30T YOLO inference failed: %s", error.c_str());
            if (frame_callback_) frame_callback_(frame.data,
                static_cast<uint32_t>(frame.total() * frame.elemSize()),
                static_cast<uint16_t>(frame.cols), static_cast<uint16_t>(frame.rows));
            continue;
        }

        // 6. 构造检测结果 JSON
        json payload;
        payload["timestampMs"] = metadata.captureTimeMs;              // 墙钟时间，展示用
        payload["monotonicTimestampMs"] = metadata.captureMonotonicMs; // 单调时钟，Localizer 对齐用
        payload["frameId"] = metadata.frameId;

        // source 映射：IR_CAM → "h30t_infrared"，ZOOM_CAM → "h30t_zoom"，其余 → "h30t_wide"
        // Localizer 用 source 判断红外/可见光参数；注意 zoom 和 wide 目前都走可见光参数
        payload["source"] = metadata.cameraSource == static_cast<uint8_t>(DJI_CAMERA_MANAGER_SOURCE_IR_CAM)
                                ? "h30t_infrared"
                                : (metadata.cameraSource == static_cast<uint8_t>(DJI_CAMERA_MANAGER_SOURCE_ZOOM_CAM)
                                       ? "h30t_zoom"
                                       : "h30t_wide");
        payload["imageWidth"] = frame.cols;
        payload["imageHeight"] = frame.rows;
        payload["inferenceMs"] = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - begin).count();  // 推理耗时
        payload["detections"] = json::array();

        // 7. 逐个检测：填 JSON + 画框 + 画标签
        for (std::size_t i = 0; i < detections.size(); ++i) {
            const YoloDetection &item = detections[i];

            // 填 JSON（字段名与 Localizer 的读取一致）
            payload["detections"].push_back({
                {"classId", item.class_id}, {"className", item.class_name},
                {"confidence", item.confidence}, {"x", item.box.x}, {"y", item.box.y},
                {"width", item.box.width}, {"height", item.box.height}});

            // 画检测框
            cv::rectangle(frame, item.box, cv::Scalar(0, 255, 0), 2);

            // 构造标签文字：类名 + 置信度（保留两位小数）
            std::ostringstream label;
            label << item.class_name << " " << std::fixed << std::setprecision(2) << item.confidence;
            int baseline = 0;
            const cv::Size text_size = cv::getTextSize(label.str(), cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);

            // 标签位置：优先放框上方，越界则贴顶/贴左
            const int text_x = std::max(0, item.box.x);
            const int text_y = std::max(text_size.height + 2, item.box.y);

            // 标签背景（实心绿矩形）
            cv::rectangle(frame, cv::Rect(text_x, text_y - text_size.height - 2,
                                          text_size.width + 4, text_size.height + 4),
                          cv::Scalar(0, 255, 0), cv::FILLED);

            // 标签文字（黑色）
            cv::putText(frame, label.str(), cv::Point(text_x + 2, text_y),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
        }

        // 8. 编码 JPEG 并回调
        std::vector<uint8_t> jpeg;
        cv::imencode(".jpg", frame, jpeg, std::vector<int>{cv::IMWRITE_JPEG_QUALITY, AppConfig::Instance().Int("minio.jpeg_quality")});

        // 回调 1：检测 JSON + JPEG（用于 MQTT 发布 / MinIO 上传）
        if (callback_) callback_(payload.dump(), jpeg);

        // 回调 2：原始标注帧（用于 RTSP 推流）
        if (frame_callback_) frame_callback_(frame.data,
            static_cast<uint32_t>(frame.total() * frame.elemSize()),
            static_cast<uint16_t>(frame.cols), static_cast<uint16_t>(frame.rows));
    }
}

/**
 * 停止服务：置停止标志、唤醒、join、清回调。
 *
 * 幂等：重复调用安全（joinable() 判断）。
 * 注意：join 会等 WorkerLoop 处理完当前帧才返回。
 */
void H30tYoloService::Stop()
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        running_ = false;
        has_frame_ = false;
        pending_frame_.release();  // 释放 Mat 引用，避免大图驻留
    }
    condition_.notify_all();  // 唤醒工作线程

    if (worker_.joinable()) worker_.join();  // 等线程退出

    callback_ = YoloResultCallback();
    frame_callback_ = YoloAnnotatedFrameCallback();
}
