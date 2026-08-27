#ifndef YOLOV8_DETECTOR_HPP
#define YOLOV8_DETECTOR_HPP

#include <opencv2/core.hpp>

#include <NvInfer.h>
#include <cuda_runtime_api.h>

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
    Yolov8Detector();
    ~Yolov8Detector();
    bool Load(const YoloDetectorConfig &config, std::string &error);
    std::vector<YoloDetection> Detect(const cv::Mat &image, std::string &error);

private:
    void Reset();

    nvinfer1::IRuntime *runtime_;
    nvinfer1::ICudaEngine *engine_;
    nvinfer1::IExecutionContext *context_;
    cudaStream_t stream_;
    int input_binding_;
    int output_binding_;
    std::vector<void *> device_bindings_;
    std::vector<float> input_buffer_;
    std::vector<float> output_buffer_;
    int output_channels_;
    int output_candidates_;
    std::vector<std::string> labels_;
    YoloDetectorConfig config_;
};

#endif
