#ifndef H30T_CONTROL_QUEUE_HPP
#define H30T_CONTROL_QUEUE_HPP

#include "h30t_types.hpp"

#include <chrono>
#include <condition_variable>
#include <deque>
#include <mutex>

struct H30tControlRequest {
    H30tControlAction action;
    int mount;
    H30tSource source;

    static H30tControlRequest Start(int mount);
    static H30tControlRequest Stop();
    static H30tControlRequest Status();
    static H30tControlRequest SwitchSource(H30tSource source);
};

class H30tControlQueue {
public:
    H30tControlQueue();
    bool Push(const H30tControlRequest &request);
    bool Pop(H30tControlRequest &request);
    bool PopFor(H30tControlRequest &request, const std::chrono::milliseconds &timeout);
    void CloseWithStop();
    H30tControlState State() const;
    bool IsClosed() const;
    void SetState(H30tControlState state);

private:
    static bool IsValid(const H30tControlRequest &request);

    mutable std::mutex mutex_;
    std::condition_variable condition_;
    std::deque<H30tControlRequest> requests_;
    bool closed_;
    H30tControlState state_;
};

#endif // H30T_CONTROL_QUEUE_HPP
