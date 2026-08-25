/**
 * @file landing_detector.cpp
 * @brief 降落悬停检测器实现
 */

#include "subscription/landing_detector.hpp"
#include "dji_logger.h"
#include "3rdparty/json.hpp"

#include <algorithm>
#include <cmath>

using json = nlohmann::json;

LandingDetector::LandingDetector(MqttBridge& mqtt, const std::string& drone_id,
                                 std::function<std::string()> flight_id_getter)
    : mqtt_(mqtt), drone_id_(drone_id), flight_id_getter_(std::move(flight_id_getter)) {
    USER_LOG_INFO("LandingDetector created");
}

void LandingDetector::onDisplayMode(uint8_t mode) {
    std::lock_guard<std::mutex> lock(mtx_);
    current_mode_ = mode;

    // 如果显示模式改变，重置对应的悬停消息标志
    if (mode != 12) hover_sent_12_ = false;
    if (mode != 17) hover_sent_17_ = false;

    USER_LOG_DEBUG("LandingDetector: display mode changed to %d", mode);
}

void LandingDetector::onFusedHeight(float height) {
    std::lock_guard<std::mutex> lock(mtx_);

    // 1. 只有显示模式为 12（自动降落）或 17（自动返航）时进行检测
    if (current_mode_ != 12 && current_mode_ != 17) {
        // 不相关模式，重置所有状态
        height_history_.clear();
        stable_count_ = 0;
        hover_sent_12_ = false;
        hover_sent_17_ = false;
        return;
    }

    // 2. 只关注低于 5 米的高度
    if (height >= 5.0f) {
        height_history_.clear();
        stable_count_ = 0;
        if (current_mode_ == 12) hover_sent_12_ = false;
        if (current_mode_ == 17) hover_sent_17_ = false;
        return;
    }

    // 3. 保存高度历史（最多 60 个，对应约 6 秒 @10Hz 或 12 秒 @5Hz，与原版一致）
    height_history_.push_back(height);
    if (height_history_.size() > 60) {
        height_history_.pop_front();
    }

    // 4. 检测高度是否稳定（连续变化小于 0.1 米）
    if (height_history_.size() >= 2) {
        float max_diff = 0.0f;
        for (size_t i = 1; i < height_history_.size(); ++i) {
            float diff = std::abs(height_history_[i] - height_history_[i - 1]);
            if (diff > max_diff) max_diff = diff;
        }

        if (max_diff < 0.1f) {
            stable_count_++;
        } else {
            stable_count_ = 0;
            if (current_mode_ == 12) hover_sent_12_ = false;
            if (current_mode_ == 17) hover_sent_17_ = false;
        }

        // 5. 连续稳定 50 次后发送消息
        if (stable_count_ >= 50) {
            std::string topic = "drone/" + drone_id_ + "/psdk/telemetry/fcsub/landing_hover";
            json j;

            // 获取当前的 flightId（从回调函数获得）
            std::string flightid = flight_id_getter_ ? flight_id_getter_() : "";
            j["flightId"] = flightid;
            j["height"] = height;

            if (current_mode_ == 12 && !hover_sent_12_) {
                j["hovering"] = 1;
                mqtt_.publish(topic, j.dump());
                hover_sent_12_ = true;
                USER_LOG_INFO("Landing hover message sent (mode=12). Height=%.2f, flightId=%s",
                              height, flightid.c_str());
            } else if (current_mode_ == 17 && !hover_sent_17_) {
                j["hovering"] = 2;
                mqtt_.publish(topic, j.dump());
                hover_sent_17_ = true;
                USER_LOG_INFO("Return hover message sent (mode=17). Height=%.2f, flightId=%s",
                              height, flightid.c_str());
            }
        }
    }
}