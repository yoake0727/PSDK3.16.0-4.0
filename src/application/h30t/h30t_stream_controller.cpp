#include "h30t_stream_controller.hpp"

#include "h30t_config.hpp"
#include "h30t_control_queue.hpp"
#include "h30t_liveview_session.hpp"

#include "dji_logger.h"

#include <chrono>
#include <mutex>
#include <sstream>
#include <thread>

class H30tStreamController::Impl {
public:
    explicit Impl(const H30tStreamAckCallback &ack)
        : ack_(ack), worker_started_(false) {}
    ~Impl() { Shutdown(); }

    bool StartWorker()
    {
        std::lock_guard<std::mutex> lock(worker_mutex_);
        if (worker_started_) return true;
        worker_started_ = true;
        worker_ = std::thread(&Impl::WorkerLoop, this);
        return true;
    }

    bool RequestStart(int mount)
    {
        if (queue_.State() != H30tControlState::kStopped) return false;
        queue_.SetState(H30tControlState::kStarting);
        if (!queue_.Push(H30tControlRequest::Start(mount))) {
            queue_.SetState(H30tControlState::kStopped); return false;
        }
        return true;
    }

    bool RequestStop()
    {
        const H30tControlState state = queue_.State();
        if (state == H30tControlState::kStopped || state == H30tControlState::kStopping)
            return false;
        queue_.SetState(H30tControlState::kStopping);
        return queue_.Push(H30tControlRequest::Stop());
    }

    bool RequestStatus() { return queue_.Push(H30tControlRequest::Status()); }
    bool RequestSource(H30tSource source)
    {
        if (queue_.State() != H30tControlState::kRunning) return false;
        return queue_.Push(H30tControlRequest::SwitchSource(source));
    }

    void Shutdown()
    {
        {
            std::lock_guard<std::mutex> lock(worker_mutex_);
            if (!worker_started_) return;
            queue_.CloseWithStop();
        }
        if (worker_.joinable()) worker_.join();
        std::lock_guard<std::mutex> lock(worker_mutex_);
        worker_started_ = false;
    }

    H30tControlState State() const { return queue_.State(); }
    void SetRgbFrameCallback(const H30tRgbFrameCallback &callback)
    { session_.SetRgbFrameCallback(callback); }
    void PushProcessedRgb(const uint8_t *data, uint32_t length, uint16_t width, uint16_t height)
    { session_.PushProcessedRgb(data, length, width, height); }

private:
    void Ack(const std::string &command, const std::string &status,
             const std::string &message)
    { if (ack_) ack_(command, status, message); }

    void WorkerLoop()
    {
        H30tControlRequest request;
        while (true) {
            if (!queue_.PopFor(request, std::chrono::milliseconds(100))) {
                session_.ServiceIntraframeRequests();
                continue;
            }
            if (request.action == H30tControlAction::kStop) {
                StopSession();
                if (queue_.IsClosed()) break;
            } else if (request.action == H30tControlAction::kStart) {
                StartSession(request.mount);
            } else if (request.action == H30tControlAction::kStatus) {
                PublishStatus();
            } else if (request.action == H30tControlAction::kSwitchSource) {
                if (!session_.SwitchSource(request.source))
                    Ack("h30t_source", "error", "source switch failed");
                else
                    Ack("h30t_source", "ok", "source switched");
            }
        }
    }

    bool StartSession(int mount)
    {
        queue_.SetState(H30tControlState::kStarting);
        const H30tRtspConfig config = h30t_config::LoadRtspConfigFromEnvironment();
        if (!h30t_config::ValidateRtspConfig(config))
            return FailStart("invalid or missing single RTSP URL");
        std::string error;
        if (!session_.Start(mount, config, error)) return FailStart(error);
        queue_.SetState(H30tControlState::kRunning);
        Ack("h30t_stream_start", "ok", "H30T RGB RTSP stream started");
        return true;
    }

    bool FailStart(const std::string &message)
    {
        queue_.SetState(H30tControlState::kFailed);
        USER_LOG_ERROR("Start H30T RGB RTSP stream failed: %s", message.c_str());
        session_.Stop();
        queue_.SetState(H30tControlState::kStopped);
        Ack("h30t_stream_start", "error", message);
        return false;
    }

    void PublishStatus()
    {
        const H30tStreamStatus stream = session_.Status();
        std::ostringstream message;
        message << "state=" << static_cast<int>(queue_.State())
                << ", rtsp=" << H30tRtspStateName(stream.rtsp_state)
                << ", dropped=" << stream.dropped_chunks;
        Ack("h30t_stream_status", "ok", message.str());
    }

    void StopSession(bool publish_ack = true)
    {
        queue_.SetState(H30tControlState::kStopping);
        session_.Stop();
        queue_.SetState(H30tControlState::kStopped);
        if (publish_ack) Ack("h30t_stream_stop", "ok", "H30T RGB RTSP stream stopped");
    }

    H30tStreamAckCallback ack_;
    H30tControlQueue queue_;
    H30tLiveviewSession session_;
    std::thread worker_;
    mutable std::mutex worker_mutex_;
    bool worker_started_;
};

H30tStreamController::H30tStreamController(const H30tStreamAckCallback &ack)
    : impl_(new Impl(ack)) {}
H30tStreamController::~H30tStreamController() {}
bool H30tStreamController::StartWorker() { return impl_->StartWorker(); }
bool H30tStreamController::RequestStart(int mount) { return impl_->RequestStart(mount); }
bool H30tStreamController::RequestStop() { return impl_->RequestStop(); }
bool H30tStreamController::RequestStatus() { return impl_->RequestStatus(); }
bool H30tStreamController::RequestSource(H30tSource source) { return impl_->RequestSource(source); }
void H30tStreamController::SetRgbFrameCallback(const H30tRgbFrameCallback &callback)
{ impl_->SetRgbFrameCallback(callback); }
void H30tStreamController::PushProcessedRgb(const uint8_t *data, uint32_t length,
                                             uint16_t width, uint16_t height)
{ impl_->PushProcessedRgb(data, length, width, height); }
void H30tStreamController::Shutdown() { impl_->Shutdown(); }
H30tControlState H30tStreamController::State() const { return impl_->State(); }
