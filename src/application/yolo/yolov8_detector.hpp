#ifndef YOLOV8_DETECTOR_HPP
#define YOLOV8_DETECTOR_HPP

#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>

#include <string>
#include <vector>

struct YoloDetection {
    int class_id;
    std::string class_name;
    float confidence;
    cv::Rect box;
};

struct YoloDetectorConfig {
    std::string model_path;
    std::string labels_path;
    int input_size;
    float confidence_threshold;
    float nms_threshold;

    YoloDetectorConfig();
};

class Yolov8Detector {
public:
    bool Load(const YoloDetectorConfig &config, std::string &error);
    std::vector<YoloDetection> Detect(const cv::Mat &image, std::string &error);

private:
    cv::dnn::Net net_;
    std::vector<std::string> labels_;
    YoloDetectorConfig config_;
};

#endif
