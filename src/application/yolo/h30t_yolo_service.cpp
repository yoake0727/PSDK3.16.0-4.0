#include "h30t_yolo_service.hpp"

#include "3rdparty/json.hpp"
#include "dji_logger.h"
#include <opencv2/imgproc.hpp>

#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <sstream>

using json = nlohmann::json;

namespace {
    // 读取环境变量，若不存在则返回默认值
float EnvFloat(const char *name, float fallback)
{ const char *value = std::getenv(name); return value ? static_cast<float>(std::atof(value)) : fallback; }
    // 读取环境变量，若不存在则返回默认值
int EnvInt(const char *name, int fallback)
{ const char *value = std::getenv(name); return value ? std::atoi(value) : fallback; }
}
// // 构造函数：初始化运行状态和帧标志
H30tYoloService::H30tYoloService() : running_(false), has_frame_(false) {}
H30tYoloService::~H30tYoloService() { Stop(); }

YoloDetectorConfig H30tYoloService::LoadConfig()
{
    YoloDetectorConfig config;
    const char *model = std::getenv("H30T_YOLO_MODEL");
    const char *labels = std::getenv("H30T_YOLO_LABELS");
    if (model) config.model_path = model;   // 从环境变量读取模型路径
    if (labels) config.labels_path = labels;// 从环境变量读取标签文件路径
    // 从环境变量读取推理参数
    config.input_size = EnvInt("H30T_YOLO_INPUT_SIZE", config.input_size);
    config.confidence_threshold = EnvFloat("H30T_YOLO_CONFIDENCE", config.confidence_threshold);
    config.nms_threshold = EnvFloat("H30T_YOLO_NMS", config.nms_threshold);
    return config;
}

bool H30tYoloService::Start(const YoloResultCallback &callback,
                            const YoloAnnotatedFrameCallback &frame_callback,
                            std::string &error)
{
    Stop();
    const YoloDetectorConfig config = LoadConfig();
    USER_LOG_INFO("H30T YOLO loading model: %s; labels: %s",
                  config.model_path.c_str(), config.labels_path.c_str());
    if (!detector_.Load(config, error)) return false; // 加载模型
    callback_ = callback;
    frame_callback_ = frame_callback;
    running_ = true;
    worker_ = std::thread(&H30tYoloService::WorkerLoop, this); // 启动工作线程
    return true;
}

void H30tYoloService::SubmitRgbFrame(const uint8_t *data, uint32_t length,
                                     uint16_t width, uint16_t height)
{
    if (!data || width == 0 || height == 0 ||
        length < static_cast<uint32_t>(width) * height * 3U) return;
    std::lock_guard<std::mutex> lock(mutex_);
    if (!running_) return;
    cv::Mat view(height, width, CV_8UC3, const_cast<uint8_t *>(data));
    pending_frame_ = view.clone();
    has_frame_ = true;
    condition_.notify_one();
}

void H30tYoloService::WorkerLoop()
{
    while (true) {
        cv::Mat frame; // 要处理的帧
        {
            // 1. 等待新帧到来
            std::unique_lock<std::mutex> lock(mutex_);
            condition_.wait(lock, [this]() { return !running_ || has_frame_; });
            // 2. 检查是否停止
            if (!running_) break;
            frame = pending_frame_;
            has_frame_ = false;
        }
        const std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        // 3. 进行推理
        std::string error;
        const std::vector<YoloDetection> detections = detector_.Detect(frame, error);
        // 4. 检查推理错误
        if (!error.empty()) {
            USER_LOG_WARN("H30T YOLO inference failed: %s", error.c_str());
            if (frame_callback_) frame_callback_(frame.data,
                static_cast<uint32_t>(frame.total() * frame.elemSize()),
                static_cast<uint16_t>(frame.cols), static_cast<uint16_t>(frame.rows));
            continue;
        }
        // 5. 构造JSON结果
        json payload;
        payload["timestampMs"] = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        payload["source"] = "h30t_infrared";
        payload["imageWidth"] = frame.cols;
        payload["imageHeight"] = frame.rows;
        payload["inferenceMs"] = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - begin).count();
        payload["detections"] = json::array();
        // 6. 将检测结果添加到JSON中
        for (std::size_t i = 0; i < detections.size(); ++i) {
            const YoloDetection &item = detections[i];
            payload["detections"].push_back({
                {"classId", item.class_id}, {"className", item.class_name},
                {"confidence", item.confidence}, {"x", item.box.x}, {"y", item.box.y},
                {"width", item.box.width}, {"height", item.box.height}});
            cv::rectangle(frame, item.box, cv::Scalar(0, 255, 0), 2);
            std::ostringstream label;
            label << item.class_name << " " << std::fixed << std::setprecision(2) << item.confidence;
            int baseline = 0;
            const cv::Size text_size = cv::getTextSize(label.str(), cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
            const int text_x = std::max(0, item.box.x);
            const int text_y = std::max(text_size.height + 2, item.box.y);
            cv::rectangle(frame, cv::Rect(text_x, text_y - text_size.height - 2,
                                          text_size.width + 4, text_size.height + 4),
                          cv::Scalar(0, 255, 0), cv::FILLED);
            cv::putText(frame, label.str(), cv::Point(text_x + 2, text_y),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
        }
        // 7. 调用回调函数（MQTT发布）
        if (callback_) callback_(payload.dump());
        if (frame_callback_) frame_callback_(frame.data,
            static_cast<uint32_t>(frame.total() * frame.elemSize()),
            static_cast<uint16_t>(frame.cols), static_cast<uint16_t>(frame.rows));
    }
}

void H30tYoloService::Stop()
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        running_ = false;
        has_frame_ = false;
        pending_frame_.release();
    }
    condition_.notify_all();
    if (worker_.joinable()) worker_.join();
    callback_ = YoloResultCallback();
    frame_callback_ = YoloAnnotatedFrameCallback();
}
