#pragma once
#include <opencv2/video/tracking.hpp>
#include <opencv2/core.hpp>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

struct SortDetection { cv::Rect2f box; std::string class_name; float confidence; };
struct SortTrack { std::string id; cv::Rect2f box; std::string class_name; float confidence; uint64_t frame_time_ms; };

class SortTracker {
public:
    SortTracker(const std::string &prefix, int minHits, int maxAge, float iouThreshold, float centerFactor);
    std::vector<SortTrack> update(const std::vector<SortDetection> &detections, uint64_t frameTimeMs);
private:
    struct Track;
    static std::vector<int> Hungarian(const std::vector<std::vector<double> > &cost);
    static float iou(const cv::Rect2f &a, const cv::Rect2f &b);
    std::vector<std::shared_ptr<Track> > tracks_;
    std::string prefix_; int min_hits_, max_age_, next_id_; float iou_threshold_, center_factor_;
};
