#ifndef H30T_LIVEVIEW_SESSION_HPP
#define H30T_LIVEVIEW_SESSION_HPP

#include "h30t_types.hpp"

#include <memory>
#include <string>

class H30tRgbStreamPipeline;

class H30tLiveviewSession {
public:
    H30tLiveviewSession();
    ~H30tLiveviewSession();

    bool Start(int mount, const H30tRtspConfig &config, std::string &error);
    void SetRgbFrameCallback(const H30tRgbFrameCallback &callback);
    void PushProcessedRgb(const uint8_t *data, uint32_t length, uint16_t width, uint16_t height);
    void ServiceIntraframeRequests();
    H30tStreamStatus Status() const;
    void Stop();
    bool SwitchSource(H30tSource source);

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

#endif // H30T_LIVEVIEW_SESSION_HPP
