#include "yolov8_detector.hpp"
#include "dji_logger.h" 

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <fstream>
#include <sstream>
#include <unistd.h>


namespace {
// 检查文件是否存在
bool FileExists(const std::string &path)
{
    std::ifstream file(path.c_str(), std::ios::binary);
    return file.good();
}
} // namespace
YoloDetectorConfig::YoloDetectorConfig()
    : model_path("/home/dji/PSDK3.16.0-4.0-8.26-yolo/src/application/yolo/runtime/yolov8n.onnx"),
      labels_path("/home/dji/PSDK3.16.0-4.0-8.26-yolo/src/application/yolo/runtime/coco.names"),
      input_size(640),
      confidence_threshold(0.25F),
      nms_threshold(0.45F)
{
}
bool Yolov8Detector::Load(const YoloDetectorConfig &config, std::string &error)
{
    config_ = config;
    // ===== 1. 验证标签文件 =====
    USER_LOG_INFO("Checking labels file: %s", config.labels_path.c_str());
    std::ifstream labels_file(config.labels_path.c_str());
    if (!labels_file.good()) {
        // 获取当前工作目录
        char cwd[1024];
        if (getcwd(cwd, sizeof(cwd)) != NULL) {
            USER_LOG_ERROR("   Current working directory: %s", cwd);
        }
        error = "cannot open YOLO labels: " + config.labels_path;
        USER_LOG_ERROR("%s", error.c_str());
        return false;
    }
    USER_LOG_INFO("Labels file found");
    // 读取标签文件
    labels_.clear(); 
    std::string line;
    while (std::getline(labels_file, line)) if (!line.empty()) labels_.push_back(line);
    USER_LOG_INFO("   Loaded %zu labels", labels_.size());
    // 3. 加载 ONNX 模型
    try {
        net_ = cv::dnn::readNetFromONNX(config.model_path);
        // DNN_BACKEND_OPENCV      # OpenCV 原生实现
        // DNN_BACKEND_INFERENCE_ENGINE  # Intel OpenVINO
        // DNN_BACKEND_CUDA        # NVIDIA CUDA
        net_.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA); 
        // DNN_TARGET_CPU          # CPU
        // DNN_TARGET_OPENCL       # OpenCL
        // DNN_TARGET_CUDA         # CUDA
        net_.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
    } catch (const cv::Exception &e) {
        error = std::string("cannot load YOLO model: ") + e.what(); 
        USER_LOG_ERROR("%s", error.c_str());
        return false;
    }
    if (net_.empty()) { error = "YOLO model is empty"; USER_LOG_ERROR("%s", error.c_str()); return false; }
    return true;
}

std::vector<YoloDetection> Yolov8Detector::Detect(const cv::Mat &image, std::string &error)
{
    std::vector<YoloDetection> result;
    // 1. 输入验证
    if (image.empty() || net_.empty()) { error = "YOLO detector or input is empty"; USER_LOG_ERROR("%s", error.c_str()); return result; }
    try {
        // 2. 图像预处理
        cv::Mat blob = cv::dnn::blobFromImage(image, 1.0 / 255.0,   // 归一化到 [0,1] 
            cv::Size(config_.input_size, config_.input_size), cv::Scalar(), false, false); // 640x640  不交换RGB通道，不裁剪  
        // 3. 设置网络输入并进行前向推理
        net_.setInput(blob);
        std::vector<cv::Mat> outputs;
        net_.forward(outputs, net_.getUnconnectedOutLayersNames());
        if (outputs.empty()) { error = "YOLO produced no output"; USER_LOG_ERROR("%s", error.c_str()); return result; }
        // 4. 解析输出 → 
        // 4.1 输出格式处理
        cv::Mat output = outputs[0];
        // YOLOv8 输出通常为 [1, 4+classes, candidates]，需要调整形状 3维->2维
        if (output.dims == 3) output = output.reshape(1, output.size[1]);
        // YOLOv8 ONNX 输出可能是 [candidates, channels] 或 [channels, candidates]
        // 如果 rows < cols 且 rows <= 256，说明需要转置
        if (output.rows < output.cols && output.rows <= 256) cv::transpose(output, output);
        // 检查输出维度
        if (output.cols < 5) { error = "unsupported YOLO output shape"; USER_LOG_ERROR("%s", error.c_str()); return result; }
        // 4.2 解析每个候选框
        const float scale_x = static_cast<float>(image.cols) / config_.input_size;
        const float scale_y = static_cast<float>(image.rows) / config_.input_size;
        std::vector<cv::Rect> boxes;
        std::vector<float> scores;
        std::vector<int> class_ids;
        for (int row = 0; row < output.rows; ++row) {
            const float *values = output.ptr<float>(row);
            // 前4个值是边界框中心点和宽高，后续是各类别的置信度 提取类别得分 (从索引 4 开始)
            cv::Mat class_scores(1, output.cols - 4, CV_32F, const_cast<float *>(values + 4));
            // 找到最大类别得分及其索引
            cv::Point class_point;
            double confidence = 0.0;
            cv::minMaxLoc(class_scores, NULL, &confidence, NULL, &class_point);
            // 置信度过滤
            if (confidence < config_.confidence_threshold) continue;
            // 计算边界框的左上角坐标和宽高
            const float width = values[2] * scale_x;
            const float height = values[3] * scale_y;
            // 边界框中心点坐标
            boxes.push_back(cv::Rect(static_cast<int>(values[0] * scale_x - width / 2),
                                     static_cast<int>(values[1] * scale_y - height / 2),
                                     static_cast<int>(width), static_cast<int>(height))
                            & cv::Rect(0, 0, image.cols, image.rows));
            scores.push_back(static_cast<float>(confidence)); // scores[0] = 第1个检测框的置信度
            class_ids.push_back(class_point.x); // class_ids[0] = 第1个检测框的类别索引
        }
        // 5. 非极大值抑制（NMS）去除重叠框
        std::vector<int> kept;
        cv::dnn::NMSBoxes(boxes, scores, config_.confidence_threshold,
                          config_.nms_threshold, kept);
        // 6. 构建检测结果
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
    } catch (const cv::Exception &e) { error = e.what(); USER_LOG_ERROR("%s", error.c_str()); }
    return result;
}
