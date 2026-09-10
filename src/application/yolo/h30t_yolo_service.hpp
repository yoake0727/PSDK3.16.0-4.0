#ifndef H30T_YOLO_SERVICE_HPP
#define H30T_YOLO_SERVICE_HPP

#include "yolov8_detector.hpp"
#include "h30t/h30t_types.hpp"

#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

typedef std::function<void(const std::string &, const std::vector<uint8_t> &)> YoloResultCallback;
typedef std::function<void(const uint8_t *, uint32_t, uint16_t, uint16_t)> YoloAnnotatedFrameCallback;

class H30tYoloService {
public:
    H30tYoloService();
    ~H30tYoloService();

    bool Start(const YoloResultCallback &callback,
               const YoloAnnotatedFrameCallback &frame_callback,
               std::string &error);
    void SubmitRgbFrame(const uint8_t *data, uint32_t length,
                        uint16_t width, uint16_t height, const H30tFrameMetadata &metadata);
    void Stop();

private:
    void WorkerLoop();
    static YoloDetectorConfig LoadConfig();

    Yolov8Detector detector_;
    YoloResultCallback callback_;
    YoloAnnotatedFrameCallback frame_callback_;
    cv::Mat pending_frame_;
    H30tFrameMetadata pending_metadata_ = {};
    std::mutex mutex_;
    std::condition_variable condition_;
    std::thread worker_;
    bool running_;
    bool has_frame_;
};

#endif
