#ifndef H30T_RGB_STREAM_PIPELINE_HPP
#define H30T_RGB_STREAM_PIPELINE_HPP

#include "h30t_types.hpp"
#include <cstddef>
#include <cstdint>
#include <memory>

class H30tRgbStreamPipeline {
public:
    H30tRgbStreamPipeline();
    ~H30tRgbStreamPipeline();
    bool Start(const H30tStreamPipelineConfig &config);
    void PushH264(const std::uint8_t *data, std::size_t length);
    H30tStreamStatus SnapshotStatus() const;
    void Stop();
private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

#endif
