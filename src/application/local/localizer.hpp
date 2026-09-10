#pragma once
#include "positioning.hpp"
#include <cstdint>
#include <mutex>
#include <string>
#include <deque>
#include "sort_tracker.hpp"

struct LocalizerConfig {
    double visible_focal_length_mm = 0.0;
    double visible_pixel_size_mm = 0.0;
    int visible_image_width = 0, visible_image_height = 0;
    double infrared_focal_length_mm = 0.0;
    double infrared_pixel_size_mm = 0.0;
    int infrared_image_width = 0, infrared_image_height = 0;
    double target_height_agl_m = 0.0;
    int sort_min_hits = 0, sort_max_age = 0;
    double sort_iou_threshold = 0.0, sort_center_factor = 0.0;
    uint64_t telemetry_match_max_diff_ms = 0;
    local_positioning::LeverArm visible_lever_arm;
    local_positioning::LeverArm infrared_lever_arm;
};

struct LocalizerTelemetry {
    uint64_t timestamp_ms = 0;
    double latitude_deg = 0.0, longitude_deg = 0.0, height_agl_m = 0.0;
    double pitch_deg = 0.0, roll_deg = 0.0, yaw_deg = 0.0;
    LocalizerTelemetry() {}
    LocalizerTelemetry(uint64_t timestamp, double latitude, double longitude,
                       double height, double pitch, double roll, double yaw)
        : timestamp_ms(timestamp), latitude_deg(latitude), longitude_deg(longitude),
          height_agl_m(height), pitch_deg(pitch), roll_deg(roll), yaw_deg(yaw) {}
};

class Localizer {
public:
    explicit Localizer(const std::string &droneId,
                       const std::string &configPath = "/home/dji/PSDK3.16.0-4.3-9.3-dingwei/src/app_json/app_config.yaml");
    void updateTelemetry(const LocalizerTelemetry &sample);
    bool processDetectionJson(const std::string &payload, std::string &locationJson);
private:
    std::string drone_id_;
    mutable std::mutex mutex_;
    LocalizerConfig config_;
    std::deque<LocalizerTelemetry> telemetry_history_;
    SortTracker tracker_;
    bool findNearestTelemetry(uint64_t timestampMs, LocalizerTelemetry &sample, uint64_t &diffMs) const;
};
