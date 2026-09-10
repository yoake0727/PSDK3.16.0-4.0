#include "video_file_source.hpp"
#include "dji_logger.h"
#include <opencv2/videoio.hpp>
#include <chrono>

namespace {

/**
 * 墙钟毫秒时间戳（system_clock）。
 * 可能被 NTP 调整，用于 metadata.captureTimeMs（展示/日志）。
 */
uint64_t WallClockMs() {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count());
}

/**
 * 单调时钟毫秒时间戳（steady_clock）。
 * 不会被系统时间调整影响，用于 metadata.captureMonotonicMs（Localizer 时间对齐）。
 */
uint64_t MonotonicMs() {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}

} // namespace

VideoFileSource::VideoFileSource() {}

/**
 * 析构：自动 Stop()，保证工作线程退出、回调清空。
 */
VideoFileSource::~VideoFileSource() { Stop(); }

/**
 * 启动本地视频源。
 *
 * 参数：
 *   path         : 视频文件路径
 *   cameraSource : 模拟的相机源（IR/ZOOM/WIDE），写入 metadata.cameraSource
 *   loop         : 是否循环播放
 *   callback     : 帧回调，参数 (data, len, w, h, metadata)
 *   error        : 输出，失败原因
 *
 * 返回：
 *   true  : 校验通过，线程已启动
 *   false : 打不开视频 / FPS 无效
 *
 * 流程：
 *   1. Stop() 清理旧状态
 *   2. 用局部 probe 探测视频能否打开、FPS 是否有效
 *   3. 保存参数
 *   4. 启动 Run 线程
 *
 * 注意：probe 只用于校验，Run() 里会重新打开一个 VideoCapture。
 */
bool VideoFileSource::Start(const std::string &path, uint8_t cameraSource, bool loop,
                            const FrameCallback &callback, std::string &error) {
    Stop();

    // 探测视频：能否打开、FPS 是否有效
    cv::VideoCapture probe(path);
    if (!probe.isOpened()) {
        error = "Cannot open video.file_path: " + path;
        return false;
    }
    if (probe.get(cv::CAP_PROP_FPS) <= 0.0) {
        error = "Video FPS is invalid";
        return false;
    }

    // 保存参数
    path_ = path;
    camera_source_ = cameraSource;
    loop_ = loop;
    callback_ = callback;

    // 启动工作线程
    running_ = true;
    worker_ = std::thread(&VideoFileSource::Run, this);
    return true;
}

/**
 * 工作线程主循环。
 *
 * 流程：
 *   1. 打开视频，取 FPS，算出每帧间隔 period
 *   2. 循环读帧
 *      - 读到末尾：loop_ 为 true 则跳到第 0 帧，否则退出
 *      - 帧无效（空/宽高超 uint16）：退出
 *      - 构造 metadata，调 callback_
 *      - sleep(period) 控制帧率
 *
 * 注意：
 *   - frame.data 是局部 Mat 的指针，回调返回后失效，回调方必须拷贝
 *   - sleep 不补偿 callback 耗时，实际帧率可能低于视频 FPS
 *   - 这里没有校验 capture.isOpened()，FPS 为 0 时会除零
 */
void VideoFileSource::Run() {
    cv::VideoCapture capture(path_);
    const double fps = capture.get(cv::CAP_PROP_FPS);

    // 每帧间隔（微秒）。例：30 FPS → 1000000/30 ≈ 33333 us
    const std::chrono::microseconds period(static_cast<long long>(1000000.0 / fps));

    uint32_t frame_id = 0;
    while (running_) {
        cv::Mat frame;

        // 读帧失败 = 到末尾或出错
        if (!capture.read(frame)) {
            if (loop_) {
                // 循环播放：跳到第 0 帧，继续
                capture.set(cv::CAP_PROP_POS_FRAMES, 0);
                continue;
            }
            USER_LOG_INFO("Local video reached end of file");
            break;
        }

        // 帧校验：空帧、宽高超出 uint16_t（因为 metadata 用 uint16_t）
        if (frame.empty() || frame.cols > UINT16_MAX || frame.rows > UINT16_MAX) {
            USER_LOG_ERROR("Invalid local video frame");
            break;
        }

        // 构造 metadata：墙钟 + 单调时钟 + 帧号 + 相机源
        const H30tFrameMetadata meta = {WallClockMs(), MonotonicMs(), frame_id++, camera_source_};

        // 回调送帧（callback 必须立即拷贝，data 生命周期只到本轮循环结束）
        if (callback_) {
            callback_(frame.data,
                      static_cast<uint32_t>(frame.total() * frame.elemSize()),
                      static_cast<uint16_t>(frame.cols),
                      static_cast<uint16_t>(frame.rows),
                      meta);
        }

        // 按视频帧率 sleep，模拟真实相机
        std::this_thread::sleep_for(period);
    }
}

/**
 * 停止视频源。
 *
 * 1. 置 running_ = false
 * 2. join 工作线程（会等当前回调返回）
 * 3. 清空回调
 *
 * 幂等：joinable() 判断保证重复调用安全。
 *
 * 注意：join 会等当前 callback_ 返回，如果回调阻塞会卡住 Stop。
 */
void VideoFileSource::Stop() {
    running_ = false;
    if (worker_.joinable()) worker_.join();
    callback_ = FrameCallback();
}