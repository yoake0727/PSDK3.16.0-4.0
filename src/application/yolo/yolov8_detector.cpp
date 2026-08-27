#include "yolov8_detector.hpp"
#include "dji_logger.h" 

#include <opencv2/imgproc.hpp>
#include <opencv2/dnn/dnn.hpp>

#include <algorithm>
#include <fstream>
#include <sstream>
#include <unistd.h>


namespace {
class TrtLogger : public nvinfer1::ILogger {
public:
    void log(Severity severity, const char *message) noexcept override
    {
        if (severity <= Severity::kWARNING) USER_LOG_WARN("TensorRT: %s", message);
    }
};

TrtLogger g_trt_logger;

std::size_t ElementCount(const nvinfer1::Dims &dims)
{
    std::size_t count = 1;
    for (int i = 0; i < dims.nbDims; ++i) {
        if (dims.d[i] <= 0) return 0;
        count *= static_cast<std::size_t>(dims.d[i]);
    }
    return count;
}

// 检查文件是否存在
bool FileExists(const std::string &path)
{
    std::ifstream file(path.c_str(), std::ios::binary);
    return file.good();
}
} // namespace

Yolov8Detector::Yolov8Detector()
    : runtime_(NULL), engine_(NULL), context_(NULL), stream_(NULL),
      input_binding_(-1), output_binding_(-1), output_channels_(0),
      output_candidates_(0) {}

Yolov8Detector::~Yolov8Detector() { Reset(); }

void Yolov8Detector::Reset()
{
    for (std::size_t i = 0; i < device_bindings_.size(); ++i) {
        if (device_bindings_[i]) cudaFree(device_bindings_[i]);
    }
    device_bindings_.clear();
    if (stream_) cudaStreamDestroy(stream_);
    stream_ = NULL;
    if (context_) context_->destroy();
    if (engine_) engine_->destroy();
    if (runtime_) runtime_->destroy();
    context_ = NULL; engine_ = NULL; runtime_ = NULL;
    input_binding_ = -1; output_binding_ = -1;
}
YoloDetectorConfig::YoloDetectorConfig()
    : model_path("/home/dji/PSDK3.16.0-4.1-8.26-yolo/src/application/yolo/runtime/yolov8n.engine"),
      labels_path("/home/dji/PSDK3.16.0-4.1-8.26-yolo/src/application/yolo/runtime/coco.names"),
      input_size(640),
      confidence_threshold(0.25F),
      nms_threshold(0.45F)
{
}
bool Yolov8Detector::Load(const YoloDetectorConfig &config, std::string &error)
{
    Reset();
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
    std::ifstream engine_file(config.model_path.c_str(), std::ios::binary | std::ios::ate);
    if (!engine_file) { error = "cannot open TensorRT engine: " + config.model_path; return false; }
    const std::streamsize engine_size = engine_file.tellg();
    engine_file.seekg(0, std::ios::beg);
    std::vector<char> serialized(static_cast<std::size_t>(engine_size));
    if (!engine_file.read(serialized.data(), engine_size)) {
        error = "cannot read TensorRT engine"; return false;
    }
    runtime_ = nvinfer1::createInferRuntime(g_trt_logger);
    if (!runtime_) { error = "createInferRuntime failed"; return false; }
    engine_ = runtime_->deserializeCudaEngine(serialized.data(), serialized.size());
    if (!engine_) { error = "deserializeCudaEngine failed"; Reset(); return false; }
    context_ = engine_->createExecutionContext();
    if (!context_) { error = "createExecutionContext failed"; Reset(); return false; }

    device_bindings_.assign(engine_->getNbBindings(), NULL);
    for (int i = 0; i < engine_->getNbBindings(); ++i) {
        if (engine_->bindingIsInput(i)) input_binding_ = i;
        else output_binding_ = i;
    }
    if (input_binding_ < 0 || output_binding_ < 0 || engine_->getNbBindings() != 2) {
        error = "TensorRT engine must have exactly one input and one output"; Reset(); return false;
    }
    nvinfer1::Dims input_dims = engine_->getBindingDimensions(input_binding_);
    if (input_dims.nbDims == 4 && input_dims.d[0] < 0) input_dims.d[0] = 1;
    if (input_dims.nbDims == 4 && input_dims.d[2] < 0) input_dims.d[2] = config.input_size;
    if (input_dims.nbDims == 4 && input_dims.d[3] < 0) input_dims.d[3] = config.input_size;
    if (!context_->setBindingDimensions(input_binding_, input_dims) ||
        !context_->allInputDimensionsSpecified()) {
        error = "unsupported dynamic TensorRT input dimensions"; Reset(); return false;
    }
    const nvinfer1::Dims output_dims = context_->getBindingDimensions(output_binding_);
    const std::size_t input_count = ElementCount(input_dims);
    const std::size_t output_count = ElementCount(output_dims);
    if (input_count == 0 || output_count == 0 || output_dims.nbDims < 2) {
        error = "invalid TensorRT binding dimensions"; Reset(); return false;
    }
    const int last = output_dims.d[output_dims.nbDims - 1];
    const int previous = output_dims.d[output_dims.nbDims - 2];
    output_channels_ = previous <= 256 ? previous : last;
    output_candidates_ = previous <= 256 ? last : previous;
    input_buffer_.resize(input_count);
    output_buffer_.resize(output_count);
    if (cudaStreamCreate(&stream_) != cudaSuccess ||
        cudaMalloc(&device_bindings_[input_binding_], input_count * sizeof(float)) != cudaSuccess ||
        cudaMalloc(&device_bindings_[output_binding_], output_count * sizeof(float)) != cudaSuccess) {
        error = "TensorRT CUDA buffer allocation failed"; Reset(); return false;
    }
    USER_LOG_INFO("TensorRT engine loaded: input=%zu, output=%dx%d",
                  input_count, output_channels_, output_candidates_);
    return true;
}

std::vector<YoloDetection> Yolov8Detector::Detect(const cv::Mat &image, std::string &error)
{
    std::vector<YoloDetection> result;
    // 1. 输入验证
    if (image.empty() || !context_) { error = "YOLO detector or input is empty"; USER_LOG_ERROR("%s", error.c_str()); return result; }
    try {
        cv::Mat resized;
        cv::resize(image, resized, cv::Size(config_.input_size, config_.input_size));
        const int plane = config_.input_size * config_.input_size;
        for (int y = 0; y < config_.input_size; ++y) {
            const cv::Vec3b *row = resized.ptr<cv::Vec3b>(y);
            for (int x = 0; x < config_.input_size; ++x) {
                const int offset = y * config_.input_size + x;
                input_buffer_[offset] = row[x][0] / 255.0F;
                input_buffer_[plane + offset] = row[x][1] / 255.0F;
                input_buffer_[2 * plane + offset] = row[x][2] / 255.0F;
            }
        }
        if (cudaMemcpyAsync(device_bindings_[input_binding_], input_buffer_.data(),
                            input_buffer_.size() * sizeof(float), cudaMemcpyHostToDevice, stream_) != cudaSuccess ||
            !context_->enqueueV2(device_bindings_.data(), stream_, NULL) ||
            cudaMemcpyAsync(output_buffer_.data(), device_bindings_[output_binding_],
                            output_buffer_.size() * sizeof(float), cudaMemcpyDeviceToHost, stream_) != cudaSuccess ||
            cudaStreamSynchronize(stream_) != cudaSuccess) {
            error = "TensorRT inference failed"; return result;
        }
        // 4.2 解析每个候选框
        const float scale_x = static_cast<float>(image.cols) / config_.input_size;
        const float scale_y = static_cast<float>(image.rows) / config_.input_size;
        std::vector<cv::Rect> boxes;
        std::vector<float> scores;
        std::vector<int> class_ids;
        const bool channel_first = output_channels_ <= 256;
        for (int row = 0; row < output_candidates_; ++row) {
            const auto value = [&](int channel) {
                return channel_first
                    ? output_buffer_[channel * output_candidates_ + row]
                    : output_buffer_[row * output_channels_ + channel];
            };
            int class_id = 0;
            float confidence = value(4);
            for (int channel = 5; channel < output_channels_; ++channel) {
                if (value(channel) > confidence) {
                    confidence = value(channel); class_id = channel - 4;
                }
            }
            // 置信度过滤
            if (confidence < config_.confidence_threshold) continue;
            // 计算边界框的左上角坐标和宽高
            const float width = value(2) * scale_x;
            const float height = value(3) * scale_y;
            // 边界框中心点坐标
            boxes.push_back(cv::Rect(static_cast<int>(value(0) * scale_x - width / 2),
                                     static_cast<int>(value(1) * scale_y - height / 2),
                                     static_cast<int>(width), static_cast<int>(height))
                            & cv::Rect(0, 0, image.cols, image.rows));
            scores.push_back(static_cast<float>(confidence)); // scores[0] = 第1个检测框的置信度
            class_ids.push_back(class_id);
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
