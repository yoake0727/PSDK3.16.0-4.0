#include "yolov8_detector.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <fstream>
#include <sstream>
#include <unistd.h>

namespace {
bool FileExists(const std::string &path)
{
    std::ifstream file(path.c_str(), std::ios::binary);
    return file.good();
}

std::string ResolveYoloAsset(const char *name)
{
    char executable[4096] = {};
    const ssize_t length = readlink("/proc/self/exe", executable, sizeof(executable) - 1);
    if (length > 0) {
        executable[length] = '\0';
        const std::string executable_path(executable);
        const std::string::size_type slash = executable_path.find_last_of('/');
        if (slash != std::string::npos) {
            const std::string deployed = executable_path.substr(0, slash) + "/../" + name;
            if (FileExists(deployed)) return deployed;
        }
    }

    const std::string source_path = std::string("src/application/yolo/") + name;
    if (FileExists(source_path)) return source_path;
    return std::string(name);
}
}

YoloDetectorConfig::YoloDetectorConfig()
    : model_path(ResolveYoloAsset("yolov8n.onnx")),
      labels_path(ResolveYoloAsset("coco.names")),
      input_size(640),
      confidence_threshold(0.25F),
      nms_threshold(0.45F)
{
}
bool Yolov8Detector::Load(const YoloDetectorConfig &config, std::string &error)
{
    config_ = config;
    std::ifstream labels(config.labels_path.c_str());
    if (!labels) { error = "cannot open YOLO labels: " + config.labels_path; return false; }
    labels_.clear();
    std::string line;
    while (std::getline(labels, line)) if (!line.empty()) labels_.push_back(line);
    try {
        net_ = cv::dnn::readNetFromONNX(config.model_path);
        net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    } catch (const cv::Exception &e) {
        error = std::string("cannot load YOLO model: ") + e.what(); return false;
    }
    if (net_.empty()) { error = "YOLO model is empty"; return false; }
    return true;
}

std::vector<YoloDetection> Yolov8Detector::Detect(const cv::Mat &image, std::string &error)
{
    std::vector<YoloDetection> result;
    if (image.empty() || net_.empty()) { error = "YOLO detector or input is empty"; return result; }
    try {
        cv::Mat blob = cv::dnn::blobFromImage(image, 1.0 / 255.0,
            cv::Size(config_.input_size, config_.input_size), cv::Scalar(), false, false);
        net_.setInput(blob);
        std::vector<cv::Mat> outputs;
        net_.forward(outputs, net_.getUnconnectedOutLayersNames());
        if (outputs.empty()) { error = "YOLO produced no output"; return result; }

        cv::Mat output = outputs[0];
        if (output.dims == 3) output = output.reshape(1, output.size[1]);
        // Standard YOLOv8 ONNX output is [1, 4+classes, candidates].
        if (output.rows < output.cols && output.rows <= 256) cv::transpose(output, output);
        if (output.cols < 5) { error = "unsupported YOLO output shape"; return result; }

        const float scale_x = static_cast<float>(image.cols) / config_.input_size;
        const float scale_y = static_cast<float>(image.rows) / config_.input_size;
        std::vector<cv::Rect> boxes;
        std::vector<float> scores;
        std::vector<int> class_ids;
        for (int row = 0; row < output.rows; ++row) {
            const float *values = output.ptr<float>(row);
            cv::Mat class_scores(1, output.cols - 4, CV_32F, const_cast<float *>(values + 4));
            cv::Point class_point;
            double confidence = 0.0;
            cv::minMaxLoc(class_scores, NULL, &confidence, NULL, &class_point);
            if (confidence < config_.confidence_threshold) continue;
            const float width = values[2] * scale_x;
            const float height = values[3] * scale_y;
            boxes.push_back(cv::Rect(static_cast<int>(values[0] * scale_x - width / 2),
                                     static_cast<int>(values[1] * scale_y - height / 2),
                                     static_cast<int>(width), static_cast<int>(height))
                            & cv::Rect(0, 0, image.cols, image.rows));
            scores.push_back(static_cast<float>(confidence));
            class_ids.push_back(class_point.x);
        }
        std::vector<int> kept;
        cv::dnn::NMSBoxes(boxes, scores, config_.confidence_threshold,
                          config_.nms_threshold, kept);
        for (std::size_t i = 0; i < kept.size(); ++i) {
            const int index = kept[i];
            YoloDetection detection;
            detection.class_id = class_ids[index];
            detection.class_name = detection.class_id >= 0 &&
                detection.class_id < static_cast<int>(labels_.size())
                ? labels_[detection.class_id] : std::to_string(detection.class_id);
            detection.confidence = scores[index];
            detection.box = boxes[index];
            result.push_back(detection);
        }
    } catch (const cv::Exception &e) { error = e.what(); }
    return result;
}
