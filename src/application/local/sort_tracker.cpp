#include "sort_tracker.hpp"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

/**
 * 单条轨迹的内部状态。
 *
 * 卡尔曼滤波状态向量（8 维）：
 *   [ cx, cy, w, h, vx, vy, vw, vh ]
 *     └─位置/尺寸─┘ └──速度──┘
 *   - (cx, cy)：框中心
 *   - (w, h)  ：框宽高
 *   - v*      ：对应量的变化率
 *
 * 观测向量（4 维）：[ cx, cy, w, h ]，只观测位置/尺寸，不观测速度。
 */
struct SortTracker::Track {
    cv::KalmanFilter filter;
    std::string id, class_name;          // id 惰性分配，class_name 构造后固定
    int hit_streak = 0;                  // 连续命中帧数（只增不减）
    int time_since_update = 0;           // 距上次成功匹配的帧数，用于老化
    float confidence = 0;                // 最近一次检测置信度

    /**
     * 构造：初始化卡尔曼滤波器，用首个检测框设置初始状态。
     *
     * 转移矩阵：恒速模型，前 4 维由后 4 维速度累加
     *   transitionMatrix(i, i+4) = 1  (i = 0..3)
     *
     * 观测矩阵：只测前 4 维（cx, cy, w, h）
     *   measurementMatrix(i, i) = 1  (i = 0..3)
     *
     * 过程噪声 Q：位置 0.01，速度 1.0（速度项大，快速目标预测更跟手）
     * 观测噪声 R：1.0
     * 初始误差协方差 P：1000（位置不确定性大）
     *
     * 初始速度默认为 0，首帧速度未知，前几帧预测会滞后。
     */
    Track(const SortDetection &d)
        : filter(8, 4, 0, CV_32F), class_name(d.class_name), confidence(d.confidence) {
        // 恒速模型：位置 += 速度
        filter.transitionMatrix = cv::Mat::eye(8, 8, CV_32F);
        for (int i = 0; i < 4; ++i) filter.transitionMatrix.at<float>(i, i + 4) = 1.0f;

        // 观测矩阵：只观测前 4 维
        filter.measurementMatrix = cv::Mat::zeros(4, 8, CV_32F);
        for (int i = 0; i < 4; ++i) filter.measurementMatrix.at<float>(i, i) = 1.0f;

        // 过程噪声：位置 0.01，速度 1.0
        cv::setIdentity(filter.processNoiseCov, cv::Scalar(0.01));
        for (int i = 4; i < 8; ++i) filter.processNoiseCov.at<float>(i, i) = 1.0f;

        // 观测噪声 1.0，初始误差协方差 1000
        cv::setIdentity(filter.measurementNoiseCov, cv::Scalar(1.0));
        cv::setIdentity(filter.errorCovPost, cv::Scalar(1000.0));

        // 用检测框初始化状态：中心 + 宽高，速度保持 0
        filter.statePost.at<float>(0) = d.box.x + d.box.width / 2;
        filter.statePost.at<float>(1) = d.box.y + d.box.height / 2;
        filter.statePost.at<float>(2) = d.box.width;
        filter.statePost.at<float>(3) = d.box.height;
    }

    /**
     * 卡尔曼预测：状态推进一帧，返回预测框。
     *
     * 注意：本函数有副作用——每调用一次 time_since_update 自增 1。
     * 外部应只在 update() 开头对每条轨迹调用一次，避免状态被破坏。
     *
     * 返回的宽高用 max(0, ...) 保护，避免滤波发散出负值。
     */
    cv::Rect2f predict() {
        cv::Mat s = filter.predict();
        ++time_since_update;
        return cv::Rect2f(s.at<float>(0) - s.at<float>(2) / 2,
                          s.at<float>(1) - s.at<float>(3) / 2,
                          std::max(0.f, s.at<float>(2)),
                          std::max(0.f, s.at<float>(3)));
    }

    /**
     * 卡尔曼校正：用检测框更新状态。
     *
     * 副作用：
     *   - time_since_update 清零
     *   - hit_streak 自增 1（只增不减）
     *   - confidence 更新为最新检测置信度
     */
    void update(const SortDetection &d) {
        cv::Mat z(4, 1, CV_32F);
        z.at<float>(0) = d.box.x + d.box.width / 2;
        z.at<float>(1) = d.box.y + d.box.height / 2;
        z.at<float>(2) = d.box.width;
        z.at<float>(3) = d.box.height;
        filter.correct(z);
        time_since_update = 0;
        ++hit_streak;
        confidence = d.confidence;
    }
};

/**
 * 构造：记录前缀、确认阈值、老化阈值、IoU 阈值、中心距离系数。
 *
 * next_id_ 从 1 开始，ID 在轨迹首次输出时惰性分配。
 */
SortTracker::SortTracker(const std::string &p, int m, int a, float i, float c)
    : prefix_(p), min_hits_(m), max_age_(a), next_id_(1),
      iou_threshold_(i), center_factor_(c) {}

/**
 * 计算两个框的 IoU（交并比）。
 *
 *   IoU = 交集面积 / 并集面积
 *
 * 并集面积为 0 时返回 0，避免除零。
 */
float SortTracker::iou(const cv::Rect2f &a, const cv::Rect2f &b) {
    float inter = (a & b).area();
    float uni = a.area() + b.area() - inter;
    return uni > 0 ? inter / uni : 0;
}

/**
 * 匈牙利算法（JV 势函数实现，O(n^3)）。
 *
 * 输入 cost：n×m 代价矩阵，越小越优。
 * 输出 ans：长度 n，ans[i] = 第 i 行匹配到的列索引，-1 表示未匹配。
 *
 * 实现要点：
 *   - 非方阵补成 N×N（N = max(n, m)），缺失项填 1e6（禁止匹配）
 *   - 使用势函数 u/v 与 way 数组，标准 JV 写法
 *   - 最后只保留 i < n 且 j < m 的有效匹配
 *
 * 注意：匈牙利要求完美匹配，1e6 的禁止项也会被强行配对，
 * 调用方需在后续用接受准则（IoU / 中心距离）显式拒绝。
 */
std::vector<int> SortTracker::Hungarian(const std::vector<std::vector<double> > &cost) {
    const int n = cost.size(), m = n ? cost[0].size() : 0, N = std::max(n, m);
    std::vector<std::vector<double> > a(N, std::vector<double>(N, 1e6));
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < m; ++j)
            a[i][j] = cost[i][j];

    std::vector<double> u(N + 1), v(N + 1);
    std::vector<int> p(N + 1), way(N + 1);

    for (int i = 1; i <= N; ++i) {
        p[0] = i;
        int j0 = 0;
        std::vector<double> minv(N + 1, 1e18);
        std::vector<char> used(N + 1, false);
        do {
            used[j0] = true;
            int i0 = p[j0], j1 = 0;
            double delta = 1e18;
            for (int j = 1; j <= N; ++j)
                if (!used[j]) {
                    double cur = a[i0 - 1][j - 1] - u[i0] - v[j];
                    if (cur < minv[j]) minv[j] = cur, way[j] = j0;
                    if (minv[j] < delta) delta = minv[j], j1 = j;
                }
            for (int j = 0; j <= N; ++j)
                if (used[j]) u[p[j]] += delta, v[j] -= delta;
                else minv[j] -= delta;
            j0 = j1;
        } while (p[j0]);
        do {
            int j1 = way[j0];
            p[j0] = p[j1];
            j0 = j1;
        } while (j0);
    }

    // 提取匹配结果：ans[i] = j，未匹配为 -1
    std::vector<int> ans(n, -1);
    for (int j = 1; j <= N; ++j)
        if (p[j] && p[j] <= n && j <= m)
            ans[p[j] - 1] = j - 1;
    return ans;
}

/**
 * 主更新流程：预测 -> 匹配 -> 更新/新建 -> 老化删除 -> 输出。
 *
 * 参数：
 *   d  : 本帧的检测列表
 *   ts : 本帧时间戳（单调时钟毫秒），写入输出的 SortTrack
 *
 * 返回：
 *   本帧"已确认且被更新"的轨迹列表（hit_streak >= min_hits_ 且 time_since_update == 0）
 */
std::vector<SortTrack> SortTracker::update(const std::vector<SortDetection> &d, uint64_t ts) {
    // ---- 1. 对所有已有轨迹做预测（纯外推，无检测也推进） ----
    std::vector<cv::Rect2f> pred;
    for (auto &t : tracks_) pred.push_back(t->predict());

    // ---- 2. 构建代价矩阵 ----
    // 代价 = 1 - IoU，越小越像；只对同类别计算，否则保持 1e6（禁止匹配）
    std::vector<std::vector<double> > cost(d.size(), std::vector<double>(tracks_.size(), 1e6));
    for (size_t i = 0; i < d.size(); ++i)
        for (size_t j = 0; j < tracks_.size(); ++j)
            if (d[i].class_name == tracks_[j]->class_name)
                cost[i][j] = 1.0 - iou(d[i].box, pred[j]);

    // ---- 3. 匈牙利求最优匹配 ----
    std::vector<int> match = Hungarian(cost);

    // ---- 4. 逐检测接受/拒绝匹配，更新或新建 ----
    for (size_t i = 0; i < d.size(); ++i) {
        int j = match[i];
        bool ok = false;
        if (j >= 0) {
            float ov = iou(d[i].box, pred[j]);
            cv::Point2f a(d[i].box.x + d[i].box.width / 2,
                          d[i].box.y + d[i].box.height / 2);
            cv::Point2f b(pred[j].x + pred[j].width / 2,
                          pred[j].y + pred[j].height / 2);
            float dist = cv::norm(a - b);
            float side = std::max(d[i].box.width, d[i].box.height);
            // 接受条件：IoU 过阈值，或（有重叠且中心距离 <= 系数 × 检测框最长边）
            // 注意：ov > 0 要求两框必须有重叠，完全不重叠的快速目标无法救回
            ok = ov > iou_threshold_ || (ov > 0 && dist <= center_factor_ * side);
        }
        if (ok) tracks_[j]->update(d[i]);
        else    tracks_.push_back(std::make_shared<Track>(d[i]));
    }

    // ---- 5. 老化删除 ----
    // time_since_update > max_age_ 的轨迹移除；短时遮挡靠这个窗口容忍
    tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
        [this](const std::shared_ptr<Track> &t) {
            return t->time_since_update > max_age_;
        }),
        tracks_.end());

    // ---- 6. 输出已确认且本帧更新过的轨迹 ----
    std::vector<SortTrack> out;
    for (auto &t : tracks_) {
        if (t->time_since_update == 0 && t->hit_streak >= min_hits_) {
            // 惰性分配 ID：首次满足输出条件时才分配，避免浪费
            if (t->id.empty()) {
                std::ostringstream s;
                s << prefix_ << std::setw(4) << std::setfill('0') << next_id_++;
                t->id = s.str();
            }
            // 用校正后的状态 statePost 输出框，比预测框更稳
            cv::Mat x = t->filter.statePost;
            out.push_back({
                t->id,
                cv::Rect2f(x.at<float>(0) - x.at<float>(2) / 2,
                           x.at<float>(1) - x.at<float>(3) / 2,
                           x.at<float>(2), x.at<float>(3)),
                t->class_name,
                t->confidence,
                ts
            });
        }
    }
    return out;
}