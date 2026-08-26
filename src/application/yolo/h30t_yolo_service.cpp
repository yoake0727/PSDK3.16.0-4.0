#include "h30t_yolo_service.hpp"

#include "3rdparty/json.hpp"
#include "dji_logger.h"

#include <chrono>
#include <cstdlib>

using json = nlohmann::json;

namespace {
float EnvFloat(const char *name, float fallback)
{ const char *value = std::getenv(name); return value ? static_cast<float>(std::atof(value)) : fallback; }
int EnvInt(const char *name, int fallback)
{ const char *value = std::getenv(name); return value ? std::atoi(value) : fallback; }
}

H30tYoloService::H30tYoloService() : running_(false), has_frame_(false) {}
H30tYoloService::~H30tYoloService() { Stop(); }

YoloDetectorConfig H30tYoloService::LoadConfig()
{
    YoloDetectorConfig config;
    const char *model = std::getenv("H30T_YOLO_MODEL");
    const char *labels = std::getenv("H30T_YOLO_LABELS");
    if (model) config.model_path = model;
    if (labels) config.labels_path = labels;
    config.input_size = EnvInt("H30T_YOLO_INPUT_SIZE", config.input_size);
    config.confidence_threshold = EnvFloat("H30T_YOLO_CONFIDENCE", config.confidence_threshold);
    config.nms_threshold = EnvFloat("H30T_YOLO_NMS", config.nms_threshold);
    return config;
}

bool H30tYoloService::Start(const YoloResultCallback &callback, std::string &error)
{
    Stop();
    const YoloDetectorConfig config = LoadConfig();
    USER_LOG_INFO("H30T YOLO loading model: %s; labels: %s",
                  config.model_path.c_str(), config.labels_path.c_str());
    if (!detector_.Load(config, error)) return false;
    callback_ = callback;
    running_ = true;
    worker_ = std::thread(&H30tYoloService::WorkerLoop, this);
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
        cv::Mat frame;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            condition_.wait(lock, [this]() { return !running_ || has_frame_; });
            if (!running_) break;
            frame = pending_frame_;
            has_frame_ = false;
        }
        const std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        std::string error;
        const std::vector<YoloDetection> detections = detector_.Detect(frame, error);
        if (!error.empty()) { USER_LOG_WARN("H30T YOLO inference failed: %s", error.c_str()); continue; }
        json payload;
        payload["timestampMs"] = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        payload["source"] = "h30t_infrared";
        payload["imageWidth"] = frame.cols;
        payload["imageHeight"] = frame.rows;
        payload["inferenceMs"] = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - begin).count();
        payload["detections"] = json::array();
        for (std::size_t i = 0; i < detections.size(); ++i) {
            const YoloDetection &item = detections[i];
            payload["detections"].push_back({
                {"classId", item.class_id}, {"className", item.class_name},
                {"confidence", item.confidence}, {"x", item.box.x}, {"y", item.box.y},
                {"width", item.box.width}, {"height", item.box.height}});
        }
        if (callback_) callback_(payload.dump());
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
}
