#include "h30t_control_queue.hpp"

H30tControlRequest H30tControlRequest::Start(int mount)
{
    H30tControlRequest request = {H30tControlAction::kStart, mount, H30tSource::kZoom};
    return request;
}

H30tControlRequest H30tControlRequest::Stop()
{
    H30tControlRequest request = {H30tControlAction::kStop, 0, H30tSource::kZoom};
    return request;
}

H30tControlRequest H30tControlRequest::Status()
{
    H30tControlRequest request = {H30tControlAction::kStatus, 0, H30tSource::kZoom};
    return request;
}

H30tControlRequest H30tControlRequest::SwitchSource(H30tSource source)
{
    H30tControlRequest request = {H30tControlAction::kSwitchSource, 0, source};
    return request;
}

H30tControlQueue::H30tControlQueue()
    : closed_(false), state_(H30tControlState::kStopped) {}

bool H30tControlQueue::Push(const H30tControlRequest &request)
{
    if (!IsValid(request)) return false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (closed_) return false;
        requests_.push_back(request);
    }
    condition_.notify_one();
    return true;
}

bool H30tControlQueue::Pop(H30tControlRequest &request)
{
    std::unique_lock<std::mutex> lock(mutex_);
    condition_.wait(lock, [this] { return closed_ || !requests_.empty(); });
    if (requests_.empty()) return false;
    request = requests_.front();
    requests_.pop_front();
    return true;
}

bool H30tControlQueue::PopFor(H30tControlRequest &request,
                              const std::chrono::milliseconds &timeout)
{
    std::unique_lock<std::mutex> lock(mutex_);
    condition_.wait_for(lock, timeout, [this] { return closed_ || !requests_.empty(); });
    if (requests_.empty()) return false;
    request = requests_.front();
    requests_.pop_front();
    return true;
}

void H30tControlQueue::CloseWithStop()
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (closed_) return;
        requests_.clear();
        requests_.push_back(H30tControlRequest::Stop());
        closed_ = true;
    }
    condition_.notify_all();
}

H30tControlState H30tControlQueue::State() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
}

bool H30tControlQueue::IsClosed() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return closed_;
}

void H30tControlQueue::SetState(H30tControlState state)
{
    std::lock_guard<std::mutex> lock(mutex_);
    state_ = state;
}

bool H30tControlQueue::IsValid(const H30tControlRequest &request)
{
    switch (request.action) {
        case H30tControlAction::kStart: return request.mount >= 1 && request.mount <= 3;
        case H30tControlAction::kStop:
        case H30tControlAction::kStatus: return true;
        case H30tControlAction::kSwitchSource: return true;
    }
    return false;
}
