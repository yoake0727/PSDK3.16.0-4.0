#ifndef H30T_TYPES_HPP
#define H30T_TYPES_HPP

#include <cstddef>
#include <cstdint>
#include <string>

enum class H30tControlState {
    kStopped,
    kStarting,
    kRunning,
    kDegraded,
    kStopping,
    kFailed
};

enum class H30tControlAction { kStart, kStop, kStatus, kSwitchSource };
enum class H30tSource { kWide, kZoom, kInfrared };

enum class H30tRtspState {
    kDisconnected,
    kConnecting,
    kWaitingForKeyframe,
    kStreaming,
    kReconnecting,
    kStopped
};

inline const char *H30tRtspStateName(H30tRtspState state)
{
    switch (state) {
        case H30tRtspState::kDisconnected: return "DISCONNECTED";
        case H30tRtspState::kConnecting: return "CONNECTING";
        case H30tRtspState::kWaitingForKeyframe: return "WAITING_FOR_KEYFRAME";
        case H30tRtspState::kStreaming: return "STREAMING";
        case H30tRtspState::kReconnecting: return "RECONNECTING";
        case H30tRtspState::kStopped: return "STOPPED";
        default: return "UNKNOWN";
    }
}

struct H30tRtspConfig {
    std::string rtsp_url;
    std::string transport;
    std::size_t max_queue_bytes;

    H30tRtspConfig()
        : rtsp_url("rtsp://127.0.0.1:8554/h30t"), transport("tcp"), max_queue_bytes(8U * 1024U * 1024U) {}
};

struct H30tStreamPipelineConfig {
    std::string rtsp_url;
    std::string rtsp_transport;
    std::size_t max_queue_bytes;

    H30tStreamPipelineConfig()
        : rtsp_url("rtsp://192.168.147.45:8554/h30t"),
          rtsp_transport("tcp"), max_queue_bytes(8U * 1024U * 1024U) {}
};

struct H30tStreamStatus {
    H30tRtspState rtsp_state;
    std::string message;
    std::size_t queue_bytes;
    std::uint64_t dropped_chunks;

    H30tStreamStatus()
        : rtsp_state(H30tRtspState::kStopped), queue_bytes(0), dropped_chunks(0) {}
};

#endif // H30T_TYPES_HPP
