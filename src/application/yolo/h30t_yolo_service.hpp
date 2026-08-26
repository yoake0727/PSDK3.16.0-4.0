#ifndef H30T_YOLO_SERVICE_HPP
#define H30T_YOLO_SERVICE_HPP

#include "yolov8_detector.hpp"

#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

typedef std::function<void(const std::string &)> YoloResultCallback;

class H30tYoloService {
public:
    H30tYoloService();
    ~H30tYoloService();

    bool Start(const YoloResultCallback &callback, std::string &error);
    void SubmitRgbFrame(const uint8_t *data, uint32_t length,
                        uint16_t width, uint16_t height);
    void Stop();

private:
    void WorkerLoop();
    static YoloDetectorConfig LoadConfig();

    Yolov8Detector detector_;
    YoloResultCallback callback_;
    cv::Mat pending_frame_;
    std::mutex mutex_;
    std::condition_variable condition_;
    std::thread worker_;
    bool running_;
    bool has_frame_;
};

#endif
