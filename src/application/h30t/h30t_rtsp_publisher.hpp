#ifndef H30T_RTSP_PUBLISHER_HPP
#define H30T_RTSP_PUBLISHER_HPP

#include "h30t_types.hpp"

#include <cstdint>
#include <memory>

class H30tRtspPublisher {
public:
    H30tRtspPublisher();
    ~H30tRtspPublisher();

    bool Open(const H30tStreamPipelineConfig &config, int width, int height);
    bool Write(const std::uint8_t *data, int size, std::int64_t timestamp, bool keyframe);
    bool IsOpen() const;
    void Close();

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

#endif // H30T_RTSP_PUBLISHER_HPP
