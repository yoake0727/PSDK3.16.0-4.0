#include "localizer.hpp"
#include "3rdparty/json.hpp"
#include <fstream>
#include <stdexcept>
using json = nlohmann::json;

namespace {

/**
 * 字符串 -> double，转换失败时返回 fallback。
 * 用于解析 YAML 数值字段，容错不抛异常。
 */
double Number(const std::string &v, double fallback) {
    try { return std::stod(v); } catch (...) { return fallback; }
}

/**
 * 字符串 -> int，转换失败时返回 fallback。
 * 用于解析 YAML 整数/布尔字段。
 */
int Integer(const std::string &v, int fallback) {
    try { return std::stoi(v); } catch (...) { return fallback; }
}

/**
 * YAML 解析器：只支持 "key: value" 扁平结构。
 *
 * 特点：
 *   - 不支持嵌套、数组、引号、多行
 *   - key 是形如 "visible.focal_length_mm" 的扁平字符串
 *   - "#" 之后视为注释
 *   - 只识别下面 if-else 白名单里的 key，其余忽略
 *
 * 解析失败（文件打不开）抛 std::runtime_error；
 * 关键相机参数缺失或非法也抛异常，保证启动阶段就能发现配置问题。
 */
void LoadYaml(const std::string &path, LocalizerConfig &c) {
    std::ifstream in(path.c_str());
    if (!in) throw std::runtime_error("Cannot open localizer YAML: " + path);

    std::string line;
    while (std::getline(in, line)) {
        // 去掉 "#" 之后的注释
        const std::size_t hash = line.find('#');
        if (hash != std::string::npos) line.resize(hash);

        // 找 "key: value" 的分隔冒号，没有则跳过本行
        const std::size_t colon = line.find(':');
        if (colon == std::string::npos) continue;

        std::string key = line.substr(0, colon);
        std::string value = line.substr(colon + 1);

        // 去掉 key / value 的前导空白（空格、Tab）
        while (!key.empty() && (key[0] == ' ' || key[0] == '\t')) key.erase(0, 1);
        while (!value.empty() && (value[0] == ' ' || value[0] == '\t')) value.erase(0, 1);

        // ---- 相机内参 ----
        if (key == "visible.focal_length_mm") c.visible_focal_length_mm = Number(value, c.visible_focal_length_mm);
        else if (key == "visible.pixel_size_mm") c.visible_pixel_size_mm = Number(value, c.visible_pixel_size_mm);
        else if (key == "visible.image_width") c.visible_image_width = Integer(value, c.visible_image_width);
        else if (key == "visible.image_height") c.visible_image_height = Integer(value, c.visible_image_height);

        else if (key == "infrared.focal_length_mm") c.infrared_focal_length_mm = Number(value, c.infrared_focal_length_mm);
        else if (key == "infrared.pixel_size_mm") c.infrared_pixel_size_mm = Number(value, c.infrared_pixel_size_mm);
        else if (key == "infrared.image_width") c.infrared_image_width = Integer(value, c.infrared_image_width);
        else if (key == "infrared.image_height") c.infrared_image_height = Integer(value, c.infrared_image_height);

        // ---- 目标高度 ----
        else if (key == "target_height_agl_m") c.target_height_agl_m = Number(value, c.target_height_agl_m);

        // ---- SORT 跟踪参数 ----
        else if (key == "tracking.min_hits") c.sort_min_hits = Integer(value, c.sort_min_hits);
        else if (key == "tracking.max_age") c.sort_max_age = Integer(value, c.sort_max_age);
        else if (key == "tracking.iou_threshold") c.sort_iou_threshold = Number(value, c.sort_iou_threshold);
        else if (key == "tracking.center_distance_factor") c.sort_center_factor = Number(value, c.sort_center_factor);

        // ---- 遥测匹配窗口 ----
        else if (key == "telemetry.match_max_diff_ms")
            c.telemetry_match_max_diff_ms = static_cast<uint64_t>(
                Integer(value, static_cast<int>(c.telemetry_match_max_diff_ms)));

        // ---- 可见光杆臂（RTK 天线 -> 吊舱光心的机体系偏移） ----
        else if (key == "visible.lever_arm.forward_m") c.visible_lever_arm.forward_m = Number(value, c.visible_lever_arm.forward_m);
        else if (key == "visible.lever_arm.right_m") c.visible_lever_arm.right_m = Number(value, c.visible_lever_arm.right_m);
        else if (key == "visible.lever_arm.down_m") c.visible_lever_arm.down_m = Number(value, c.visible_lever_arm.down_m);

        // ---- 红外杆臂（与可见光镜头位置不同，单独标定） ----
        else if (key == "infrared.lever_arm.forward_m") c.infrared_lever_arm.forward_m = Number(value, c.infrared_lever_arm.forward_m);
        else if (key == "infrared.lever_arm.right_m") c.infrared_lever_arm.right_m = Number(value, c.infrared_lever_arm.right_m);
        else if (key == "infrared.lever_arm.down_m") c.infrared_lever_arm.down_m = Number(value, c.infrared_lever_arm.down_m);
    }

    // ---- 启动校验：8 个必填相机参数必须有效 ----
    // 缺任何一个都无法做像素->GPS 解算，直接抛异常阻止启动
    if (c.visible_focal_length_mm <= 0.0 || c.visible_pixel_size_mm <= 0.0 ||
        c.visible_image_width <= 0 || c.visible_image_height <= 0 ||
        c.infrared_focal_length_mm <= 0.0 || c.infrared_pixel_size_mm <= 0.0 ||
        c.infrared_image_width <= 0 || c.infrared_image_height <= 0 ||
        c.sort_min_hits <= 0 || c.sort_max_age <= 0 ||
        c.sort_iou_threshold <= 0.0 || c.sort_center_factor <= 0.0 ||
        c.telemetry_match_max_diff_ms == 0) {
        throw std::runtime_error("Localizer YAML contains missing or invalid camera parameters: " + path);
    }
}

} // namespace

/**
 * 构造函数。
 *
 * 注意：tracker_ 在初始化列表里先用占位默认值构造，函数体内再用
 * YAML 配置重建一次（覆盖占位值）。功能正确，但多一次构造开销。
 *
 * ID 前缀固定为 "target_inf_"，输出形如 target_inf_0001。
 *
 * 配置来源：固定项目配置文件路径。
 */
Localizer::Localizer(const std::string &droneId, const std::string &configPath)
    : drone_id_(droneId), tracker_("target_inf_", 3, 8, 0.2f, 1.0f) {
    LoadYaml(configPath, config_);
    tracker_ = SortTracker("target_inf_",
                           config_.sort_min_hits,
                           config_.sort_max_age,
                           static_cast<float>(config_.sort_iou_threshold),
                           static_cast<float>(config_.sort_center_factor));
}

/**
 * 追加一条遥测样本到缓存。
 *
 * - 加锁保护 telemetry_history_
 * - 超过 500 条弹出最旧（约 50 秒 @10Hz）
 * - 由 TelemetryPosPublisher 的飞控回调线程调用（异步）
 */
void Localizer::updateTelemetry(const LocalizerTelemetry &sample) {
    std::lock_guard<std::mutex> lock(mutex_);
    telemetry_history_.push_back(sample);
    while (telemetry_history_.size() > 500) telemetry_history_.pop_front();
}

/**
 * 在遥测缓存中查找与 ts 时间最近的一条样本。
 *
 * 参数：
 *   ts    : 目标时间戳（单调时钟毫秒）
 *   out   : 输出，最近的一条遥测（即使返回 false 也有效）
 *   diff  : 输出，最近样本与 ts 的绝对时间差（毫秒）
 *
 * 返回：
 *   true  : 找到且 diff <= telemetry_match_max_diff_ms
 *   false : 缓存为空，或 diff 超过容忍窗口
 *
 * 说明：
 *   - 纯最近邻，不做线性插值；遥测 10Hz + 检测 15Hz 时每 1~2 帧共用一条
 *   - diff 是绝对值，不区分遥测早于还是晚于帧
 *   - 线性扫描 500 条；因缓存按时间有序，理论上可二分优化
 */
bool Localizer::findNearestTelemetry(uint64_t ts, LocalizerTelemetry &out, uint64_t &diff) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (telemetry_history_.empty()) return false;

    diff = UINT64_MAX;
    for (const auto &s : telemetry_history_) {
        uint64_t d = s.timestamp_ms > ts ? s.timestamp_ms - ts : ts - s.timestamp_ms;
        if (d < diff) { diff = d; out = s; }
    }
    return diff <= config_.telemetry_match_max_diff_ms;
}

/**
 * 处理一帧检测 JSON，输出定位结果 JSON。
 *
 * 流程：
 *   1. 解析 payload，取 detections 数组和单调时间戳 frameTime
 *   2. 在遥测缓存中找 frameTime 最近的样本
 *   3. 按 source 选红外/可见光内参与杆臂
 *   4. 构造 SortDetection 列表，调用 SORT 跟踪
 *   5. 用遥测 + 杆臂计算吊舱光心位置
 *   6. 对每条已确认轨迹，做像素->GPS 解算，组装结果
 *   7. 输出 {droneId, source, locations}
 *
 * 返回：
 *   true  : locations 非空
 *   false : 解析失败 / 缺 detections / 遥测未匹配 / 无已确认轨迹
 *
 * 注意：
 *   - 遥测未匹配时直接 return false，不推进 tracker（轨迹老化停滞）
 *   - pixelToGpsWithYaw 只传了 11 个参数，pitch/roll/cameraPitch 全用默认 0
 *   - yawDeg 传的是 t.yaw_deg（机体 yaw），不是 gimbal.yaw_deg
 *   - 异常被 catch(...) 全部吞掉，无法区分失败原因
 */
bool Localizer::processDetectionJson(const std::string &payload, std::string &locationJson)
{
    try {
        // ---- 1. 解析输入 ----
        const json input = json::parse(payload);
        if (!input.contains("detections") || !input["detections"].is_array()) return false;

        // frameTime 用单调时钟，与遥测缓存的 timestamp_ms 同源，可比较
        const uint64_t frameTime = input.value("monotonicTimestampMs", uint64_t(0));

        // ---- 2. 查最近遥测 ----
        // 失败直接返回，不推进 tracker（隐患：遥测缺失期间轨迹不老化）
        LocalizerTelemetry t;
        uint64_t timeDiff = 0;
        if (!findNearestTelemetry(frameTime, t, timeDiff)) return false;

        // ---- 3. 选相机参数 ----
        // source 拼写必须精确匹配 "h30t_infrared"，否则默认走可见光
        // width/height 优先用 JSON 里的，缺失才回退 config（可能与 source 错配）
        const bool infrared = input.value("source", std::string()) == "h30t_infrared";
        const double focal = infrared ? config_.infrared_focal_length_mm : config_.visible_focal_length_mm;
        const double pixel = infrared ? config_.infrared_pixel_size_mm : config_.visible_pixel_size_mm;
        const int width = input.value("imageWidth", infrared ? config_.infrared_image_width : config_.visible_image_width);
        const int height = input.value("imageHeight", infrared ? config_.infrared_image_height : config_.visible_image_height);
        const local_positioning::LeverArm arm = infrared ? config_.infrared_lever_arm : config_.visible_lever_arm;

        // ---- 4. 构造检测并跟踪 ----
        std::vector<SortDetection> detections;
        for (const json &d : input["detections"]) {
            detections.push_back({
                cv::Rect2f(d.value("x", 0.f), d.value("y", 0.f),
                           d.value("width", 0.f), d.value("height", 0.f)),
                d.value("className", std::string()),
                d.value("confidence", 0.f)
            });
        }
        // 返回的是"已确认且本帧更新过"的轨迹（hit_streak >= min_hits）
        const std::vector<SortTrack> tracks = tracker_.update(detections, frameTime);

        // ---- 5. 计算吊舱光心位置 ----
        // 只依赖遥测和杆臂，与像素无关，所有轨迹共用
        const local_positioning::GimbalPosition gimbal = local_positioning::computeGimbalPosition(
            t.latitude_deg, t.longitude_deg, t.height_agl_m,
            t.yaw_deg, t.pitch_deg, t.roll_deg, arm);

        // ---- 6. 逐轨迹解算 GPS ----
        json results = json::array();
        for (const SortTrack &track : tracks) {
            // 取框中心作为目标像素坐标
            const double cx = track.box.x + track.box.width / 2.0;
            const double cy = track.box.y + track.box.height / 2.0;

            // 注意：这里只传了 11 个参数，pitchDeg/rollDeg/cameraPitchDeg 用默认 0
            // 后果：无人机俯仰、横滚、相机安装角全部丢失，仅悬停垂直下视时近似正确
            const local_positioning::GeoPosition target = local_positioning::pixelToGpsWithYaw(
                gimbal.latitude_deg, gimbal.longitude_deg, gimbal.height_agl_m,
                config_.target_height_agl_m,
                focal, pixel, width, height,
                cx, cy,
                t.yaw_deg);

            results.push_back({
                {"targetId", track.id},
                {"className", track.class_name},
                {"confidence", track.confidence},
                {"latitude", target.latitude_deg},
                {"longitude", target.longitude_deg},
                {"pixelX", cx},
                {"pixelY", cy},
                // timestampMs 优先用 JSON 的墙钟，缺失回退遥测时间；
                // 与 frameTime（单调）语义不同，下游需注意
                {"timestampMs", input.value("timestampMs", t.timestamp_ms)},
                {"telemetryTimeDiffMs", timeDiff}
            });
        }

        // ---- 7. 输出 ----
        // 即使 results 为空也会写入 locationJson，调用方需结合返回值判断
        locationJson = json{
            {"droneId", drone_id_},
            {"source", input.value("source", "unknown")},
            {"locations", results}
        }.dump();
        return !results.empty();

    } catch (...) {
        // 捕获所有异常：JSON 解析错、字段类型错、几何计算抛的 domain_error 等
        // 问题：不记录任何信息，定位失败时无法区分原因
        return false;
    }
}
