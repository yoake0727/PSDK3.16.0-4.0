#include "h30t_rgb_stream_pipeline.hpp"
#include "h30t_rtsp_publisher.hpp"
#include "h30t_config.hpp"
#include "dji_liveview.h"
#include "dji_logger.h"

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <vector>

class H30tRgbStreamPipeline::Impl {
public:
    Impl() : running(false), has_frame(false), dropped(0), width(0), height(0), pts(0), opened(false) {}
    std::atomic<bool> running;
    bool has_frame;
    std::uint64_t dropped;
    std::vector<std::uint8_t> rgb;
    std::uint16_t width, height;
    std::int64_t pts;
    bool opened;
    std::mutex mutex;
    std::condition_variable condition;
    std::thread worker;
    H30tStreamPipelineConfig config;
    H30tStreamStatus status;
    H30tRtspPublisher publisher;
    static Impl *active;

    void SetStatus(H30tRtspState state, const std::string &message) {
        std::lock_guard<std::mutex> lock(mutex);
        status.rtsp_state = state; status.message = message;
        status.queue_bytes = has_frame ? rgb.size() : 0;
    }
    static void EncoderCallback(const std::uint8_t *data, std::uint32_t length) {
        if (!active || !active->running.load() || !data || length == 0) return;
        std::lock_guard<std::mutex> lock(active->mutex);
        if (!active->opened) {
            active->opened = active->publisher.Open(active->config, 1440, 1080);
            if (!active->opened) {
                active->status.rtsp_state = H30tRtspState::kReconnecting;
                active->status.message = "Open RGB RTSP failed";
                return;
            }
        }
        if (!active->publisher.Write(data, static_cast<int>(length), active->pts++ * 3600, false)) {
            active->publisher.Close(); active->opened = false; active->status.rtsp_state = H30tRtspState::kReconnecting; active->status.message = "Write RGB RTSP failed"; return;
        }
        active->status.rtsp_state = H30tRtspState::kStreaming; active->status.message = "Selected-source RGB H.264 RTSP active";
    }
    void Run() {
        USER_LOG_INFO("H30T RGB pipeline waiting for first frame");
        std::vector<std::uint8_t> input;
        { std::unique_lock<std::mutex> lock(mutex); condition.wait(lock, [this]{ return !running.load() || has_frame; }); if (!running.load()) return; input.swap(rgb); has_frame=false; }
        active = this;
        opened = publisher.Open(config, 1440, 1080);
        if (!opened) { active = NULL; running = false; SetStatus(H30tRtspState::kReconnecting, "Open H.264 RTSP failed"); return; }
        SetStatus(H30tRtspState::kConnecting, "PSDK encoding selected-source RGB frames");
        while (running.load()) {
            if (input.empty()) { std::unique_lock<std::mutex> lock(mutex); condition.wait(lock, [this]{ return !running.load() || has_frame; }); if (!running.load()) break; input.swap(rgb); has_frame=false; }
            const bool keyframe = input.size() > 4 &&
                ((input[0] == 0 && input[1] == 0 && input[2] == 1 && (input[3] & 0x1F) == 5) ||
                 (input[0] == 0 && input[1] == 0 && input[2] == 0 && input[3] == 1 && (input[4] & 0x1F) == 5));
            if (!publisher.Write(input.data(), static_cast<int>(input.size()), pts++ * 3600, keyframe)) {
                publisher.Close(); opened = false; SetStatus(H30tRtspState::kReconnecting, "Write H.264 RTSP failed");
            } else {
                SetStatus(H30tRtspState::kStreaming, "H30T H.264 RTSP active");
            }
            input.clear();
        }
        publisher.Close(); opened=false; active=NULL; SetStatus(H30tRtspState::kStopped, "Selected-source H.264 pipeline stopped");
    }
};

H30tRgbStreamPipeline::Impl *H30tRgbStreamPipeline::Impl::active = NULL;
H30tRgbStreamPipeline::H30tRgbStreamPipeline() : impl_(new Impl) {}
H30tRgbStreamPipeline::~H30tRgbStreamPipeline() { Stop(); }
bool H30tRgbStreamPipeline::Start(const H30tStreamPipelineConfig &config) { if (impl_->running.load() || !h30t_config::IsValidRtspUrl(config.rtsp_url)) return false; impl_.reset(new Impl); impl_->config=config; impl_->running=true; impl_->worker=std::thread(&Impl::Run, impl_.get()); return true; }
void H30tRgbStreamPipeline::PushH264(const std::uint8_t *data, std::size_t length) { if (!data || length == 0) return; std::lock_guard<std::mutex> lock(impl_->mutex); if (impl_->has_frame) ++impl_->dropped; impl_->rgb.assign(data,data+length); impl_->has_frame=true; impl_->condition.notify_one(); }
H30tStreamStatus H30tRgbStreamPipeline::SnapshotStatus() const { std::lock_guard<std::mutex> lock(impl_->mutex); H30tStreamStatus result=impl_->status; result.dropped_chunks=impl_->dropped; return result; }
void H30tRgbStreamPipeline::Stop() { if (!impl_) return; impl_->running=false; impl_->condition.notify_all(); if (impl_->worker.joinable()) impl_->worker.join(); }
