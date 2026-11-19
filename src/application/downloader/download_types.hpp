#ifndef H30T_DOWNLOAD_TYPES_HPP
#define H30T_DOWNLOAD_TYPES_HPP

#include <cstdint>
#include <functional>
#include <string>

// H30T 原图下载状态机（评审稿 §3）
//   IDLE -> SHOOT -> SWITCHING -> DOWNLOAD -> RESUME -> SHOOT
enum class H30tDownloadState {
    kIdle,       // 未拍摄、未下载
    kShoot,      // 相机处于定时间隔拍照
    kSwitching,  // 切换到 MEDIA_DOWNLOAD，耗时 1-2 s
    kDownload,   // 分页拉列表并逐个下载
    kResume,     // 释放下载权并切回拍照模式
    kFailed      // 下载链路异常（需要人工介入或下次重试）
};

inline const char *H30tDownloadStateName(H30tDownloadState state)
{
    switch (state) {
        case H30tDownloadState::kIdle: return "IDLE";
        case H30tDownloadState::kShoot: return "SHOOT";
        case H30tDownloadState::kSwitching: return "SWITCHING";
        case H30tDownloadState::kDownload: return "DOWNLOAD";
        case H30tDownloadState::kResume: return "RESUME";
        case H30tDownloadState::kFailed: return "FAILED";
        default: return "UNKNOWN";
    }
}

// 下载模块配置，全部可由环境变量覆盖（见 download_config.cpp）
struct H30tDownloadConfig {
    std::string root_dir;       // 默认 /data/h30t
    std::string dl_dir;         // 下载中的 .part，不能与 incoming 同目录
    std::string incoming_dir;   // 成品原图，grabber 的 config.input
    std::string archive_dir;    // 可选归档目录
    std::string manifest_path;  // 下载台账 manifest.jsonl

    int mount;                  // 1..3，对应 E_DjiMountPosition
    int shoot_interval_ms;      // 定时拍照间隔，最小 2000
    int download_interval_ms;   // 自动下载批次间隔
    int max_files_per_batch;    // 每批最多下载的照片数
    int slice_count;            // 分页拉列表的每页数量，60 或 120
    int capture_lens_mask;      // 0=keep camera setting, 1=wide, 2=zoom, 4=IR
    int retry_limit;            // 单文件失败重试次数（不含首次）

    std::uint64_t min_free_bytes;   // 本机磁盘剩余阈值，低于则停止下载
    std::uint64_t min_camera_mb;    // 相机 SD 卡剩余阈值（MB）

    bool verify_jpeg;           // 发布前校验 JPEG 结构
    bool archive_after_download;// 发布到 incoming 后是否创建 archive 硬链接
    bool auto_resume_shoot;     // 下载完成后自动恢复定时拍照
    bool init_camera_manager;   // 是否由本模块 Init/DeInit 相机管理器

    H30tDownloadConfig()
        : root_dir("/data/h30t"),
          dl_dir("/data/h30t/dl"),
          incoming_dir("/data/h30t/incoming"),
          archive_dir("/data/h30t/archive"),
          manifest_path("/data/h30t/manifest.jsonl"),
          mount(1),
          shoot_interval_ms(7000),
          download_interval_ms(30000),
          max_files_per_batch(5),
          slice_count(60),
          capture_lens_mask(0),
          retry_limit(3),
          min_free_bytes(20ULL * 1024ULL * 1024ULL * 1024ULL),
          min_camera_mb(1024ULL),
          verify_jpeg(true),
          archive_after_download(false),
          auto_resume_shoot(true),
          init_camera_manager(false) {}
};

// 累计统计，供 status 命令与日志使用
struct H30tDownloadStats {
    std::uint32_t downloaded;
    std::uint32_t failed;
    std::uint32_t skipped;
    std::uint64_t bytes;
    double last_rate_bytes_per_sec;

    H30tDownloadStats()
        : downloaded(0), failed(0), skipped(0), bytes(0),
          last_rate_bytes_per_sec(0.0) {}
};

// ack(action, status, message)，status 取 "ok" / "error"
using H30tDownloadAckCallback =
    std::function<void(const std::string &, const std::string &, const std::string &)>;

#endif // H30T_DOWNLOAD_TYPES_HPP
