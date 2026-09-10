#pragma once
#include "h30t/h30t_types.hpp"
#include <atomic>
#include <functional>
#include <string>
#include <thread>

class VideoFileSource {
public:
    typedef std::function<void(const uint8_t *, uint32_t, uint16_t, uint16_t, const H30tFrameMetadata &)> FrameCallback;
    VideoFileSource();
    ~VideoFileSource();
    bool Start(const std::string &path, uint8_t cameraSource, bool loop, const FrameCallback &callback, std::string &error);
    void Stop();
private:
    void Run();
    std::string path_;
    uint8_t camera_source_ = 0;
    bool loop_ = false;
    FrameCallback callback_;
    std::atomic<bool> running_{false};
    std::thread worker_;
};
