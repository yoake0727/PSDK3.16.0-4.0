#ifndef H30T_STREAM_CONTROLLER_HPP
#define H30T_STREAM_CONTROLLER_HPP

#include "h30t_types.hpp"

#include <functional>
#include <memory>
#include <string>

typedef std::function<void(const std::string &,
                           const std::string &,
                           const std::string &)> H30tStreamAckCallback;

class H30tStreamController {
public:
    explicit H30tStreamController(const H30tStreamAckCallback &ack);
    ~H30tStreamController();

    bool StartWorker();
    bool RequestStart(int mount);
    bool RequestStop();
    bool RequestStatus();
    bool RequestSource(H30tSource source);
    void SetRgbFrameCallback(const H30tRgbFrameCallback &callback);
    void PushProcessedRgb(const uint8_t *data, uint32_t length, uint16_t width, uint16_t height);
    void Shutdown();
    H30tControlState State() const;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

#endif // H30T_STREAM_CONTROLLER_HPP
