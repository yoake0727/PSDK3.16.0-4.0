#include "h30t_downloader.hpp"

#include "download_config.hpp"
#include "download_manifest.hpp"
#include "exif_writer.hpp"

#include "dji_camera_manager.h"
#include "dji_logger.h"
#include "dji_typedef.h"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <deque>
#include <limits>
#include <mutex>
#include <sstream>
#include <thread>
#include <vector>

#include <dirent.h>
#include <sys/stat.h>
#include <sys/statvfs.h>
#include <unistd.h>

namespace {

const char kAckAction[] = "h30t_download";

E_DjiMountPosition ToMountPosition(int mount)
{
    switch (mount) {
        case 1: return DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1;
        case 2: return DJI_MOUNT_POSITION_PAYLOAD_PORT_NO2;
        case 3: return DJI_MOUNT_POSITION_PAYLOAD_PORT_NO3;
        default: return DJI_MOUNT_POSITION_UNKNOWN;
    }
}

std::string SdkError(const char *message, T_DjiReturnCode code)
{
    std::ostringstream out;
    out << message << " (SDK code=0x" << std::hex
        << static_cast<unsigned long long>(code) << ")";
    return out.str();
}

std::string JoinPath(const std::string &base, const std::string &name)
{
    return h30t_download_config::JoinPath(base, name);
}

bool EnsureDirectory(const std::string &path, std::string *error)
{
    if (path.empty()) {
        *error = "empty directory path";
        return false;
    }

    std::string current;
    std::size_t position = 0;
    if (path[0] == '/') {
        current = "/";
        position = 1;
    }

    while (position <= path.size()) {
        const std::size_t next = path.find('/', position);
        const std::string part = path.substr(
            position, next == std::string::npos ? std::string::npos : next - position);
        if (!part.empty()) {
            if (!current.empty() && current[current.size() - 1] != '/') current += '/';
            current += part;

            struct stat info;
            if (::stat(current.c_str(), &info) != 0) {
                if (::mkdir(current.c_str(), 0755) != 0 && errno != EEXIST) {
                    *error = "create directory failed: " + current;
                    return false;
                }
            } else if (!S_ISDIR(info.st_mode)) {
                *error = "path exists but is not a directory: " + current;
                return false;
            }
        }
        if (next == std::string::npos) break;
        position = next + 1;
    }
    return true;
}

bool FreeBytes(const std::string &path, std::uint64_t *free_bytes)
{
    struct statvfs info;
    if (::statvfs(path.c_str(), &info) != 0) return false;
    *free_bytes = static_cast<std::uint64_t>(info.f_bavail) * info.f_frsize;
    return true;
}

bool EndsWith(const std::string &text, const char *suffix)
{
    const std::size_t length = std::strlen(suffix);
    return text.size() >= length && text.compare(text.size() - length, length, suffix) == 0;
}

void CleanupPartFiles(const std::string &dir)
{
    DIR *handle = ::opendir(dir.c_str());
    if (handle == NULL) return;

    std::size_t removed = 0;
    struct dirent *entry = NULL;
    while ((entry = ::readdir(handle)) != NULL) {
        const std::string name(entry->d_name);
        if (name == "." || name == "..") continue;
        if (!EndsWith(name, ".part") && !EndsWith(name, ".stamped")) continue;
        if (::unlink(JoinPath(dir, name).c_str()) == 0) ++removed;
    }
    ::closedir(handle);

    if (removed > 0) {
        USER_LOG_INFO("[H30T][download] cleaned %zu stale partial file(s) in %s",
                      removed, dir.c_str());
    }
}

// 文件列表由 PSDK 通过 OSAL 分配，子文件列表单独分配，需要逐一释放
void ReleaseFileList(T_DjiCameraManagerFileList *list)
{
    if (list == NULL) return;
    if (list->fileListInfo != NULL) {
        for (uint16_t i = 0; i < list->totalCount; ++i) {
            if (list->fileListInfo[i].subFileListInfo != NULL) {
                std::free(list->fileListInfo[i].subFileListInfo);
            }
        }
        std::free(list->fileListInfo);
        list->fileListInfo = NULL;
    }
    list->totalCount = 0;
}

std::string FormatBytes(std::uint64_t bytes)
{
    char buffer[64];
    if (bytes >= 1024ULL * 1024ULL * 1024ULL) {
        std::snprintf(buffer, sizeof(buffer), "%.1f GB",
                      static_cast<double>(bytes) / (1024.0 * 1024.0 * 1024.0));
    } else {
        std::snprintf(buffer, sizeof(buffer), "%.1f MB",
                      static_cast<double>(bytes) / (1024.0 * 1024.0));
    }
    return std::string(buffer);
}

// 相机文件列表里的关键字段副本，避免持有 SDK 分配的列表内存
struct MediaItem {
    std::string name;
    std::string path;
    std::uint32_t index;
    std::uint32_t size;
    int year, month, day, hour, minute, second;

    MediaItem()
        : index(0), size(0), year(0), month(0), day(0), hour(0), minute(0), second(0) {}
};

bool MediaItemLess(const MediaItem &lhs, const MediaItem &rhs)
{
    if (lhs.year != rhs.year) return lhs.year < rhs.year;
    if (lhs.month != rhs.month) return lhs.month < rhs.month;
    if (lhs.day != rhs.day) return lhs.day < rhs.day;
    if (lhs.hour != rhs.hour) return lhs.hour < rhs.hour;
    if (lhs.minute != rhs.minute) return lhs.minute < rhs.minute;
    if (lhs.second != rhs.second) return lhs.second < rhs.second;
    return lhs.index < rhs.index;
}

// 命名规则：YYYYMMDD_HHMMSS_DJI_<index>.JPG
// 字典序即时间序，换卡后编号冲突也不会与旧文件同名
std::string BuildFinalName(const MediaItem &item)
{
    char buffer[96];
    if (item.year >= 2000 && item.month >= 1 && item.month <= 12
        && item.day >= 1 && item.day <= 31) {
        std::snprintf(buffer, sizeof(buffer), "%04d%02d%02d_%02d%02d%02d_DJI_%04u.JPG",
                      item.year, item.month, item.day, item.hour, item.minute, item.second,
                      static_cast<unsigned>(item.index));
        return std::string(buffer);
    }

    // 相机时间不可用时退化为本机时间，保证文件名仍然单调
    const time_t now = std::time(NULL);
    struct tm local;
    std::memset(&local, 0, sizeof(local));
    localtime_r(&now, &local);
    std::snprintf(buffer, sizeof(buffer), "%04d%02d%02d_%02d%02d%02d_DJI_%04u.JPG",
                  local.tm_year + 1900, local.tm_mon + 1, local.tm_mday,
                  local.tm_hour, local.tm_min, local.tm_sec,
                  static_cast<unsigned>(item.index));
    return std::string(buffer);
}

} // namespace

class H30tDownloader::Impl {
public:
    static Impl *s_active;

    explicit Impl(const H30tDownloadAckCallback &ack)
        : ack_(ack),
          state_(H30tDownloadState::kIdle),
          shutting_down_(false),
          file_(NULL),
          expected_bytes_(0),
          received_(0),
          transfer_started_(false),
          transfer_complete_(false),
          camera_manager_inited_(false),
          callback_registered_(false),
          rights_held_(false),
          shooting_(false),
          auto_enabled_(false),
          shoot_interval_ms_(config_.shoot_interval_ms),
          capture_lens_mask_(config_.capture_lens_mask),
          verify_jpeg_(config_.verify_jpeg),
          auto_resume_shoot_(config_.auto_resume_shoot),
          archive_after_download_(config_.archive_after_download)
    {
        std::memset(&stats_, 0, sizeof(stats_));
    }

    ~Impl() { Shutdown(); }

    bool Start(const H30tDownloadConfig &config, std::string *error)
    {
        std::string local_error;
        std::string *err = error ? error : &local_error;
        if (!h30t_download_config::Validate(config, err)) return false;
        config_ = config;
        auto_enabled_.store(false);
        shoot_interval_ms_.store(config.shoot_interval_ms);
        capture_lens_mask_.store(config.capture_lens_mask);
        verify_jpeg_.store(config.verify_jpeg);
        auto_resume_shoot_.store(config.auto_resume_shoot);
        archive_after_download_.store(config.archive_after_download);

        if (!EnsureDirectory(config_.root_dir, err)) return false;
        if (!EnsureDirectory(config_.dl_dir, err)) return false;
        if (!EnsureDirectory(config_.incoming_dir, err)) return false;
        if (config_.archive_after_download && !EnsureDirectory(config_.archive_dir, err)) {
            return false;
        }

        manifest_.SetPath(config_.manifest_path);

        FILE *fp = std::fopen(config_.manifest_path.c_str(), "a");
        if (fp == NULL) {
            *err = "create manifest failed: " + config_.manifest_path;
            return false;
        }
        std::fclose(fp);

        std::string manifest_error;
        if (!manifest_.Load(&manifest_error)) {
            USER_LOG_WARN("[H30T][download] %s, starting with empty manifest",
                          manifest_error.c_str());
        }

        CleanupPartFiles(config_.dl_dir);

        if (config_.init_camera_manager) {
            const T_DjiReturnCode rc = DjiCameraManager_Init();
            if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                *err = SdkError("camera manager initialization failed", rc);
                return false;
            }
            camera_manager_inited_ = true;
        } else {
            USER_LOG_INFO("[H30T][download] camera manager init is owned by the H30T stream module");
        }

        s_active = this;
        shutting_down_.store(false);
        worker_ = std::thread(&Impl::WorkerLoop, this);

        USER_LOG_INFO("[H30T][download] service started, incoming=%s manifest=%s",
                      config_.incoming_dir.c_str(), config_.manifest_path.c_str());
        Ack(kAckAction, "ok", "downloader started");
        return true;
    }

    void Shutdown()
    {
        {
            std::lock_guard<std::mutex> lock(command_mutex_);
            if (shutting_down_.exchange(true)) return;
            commands_.clear();
        }
        command_cv_.notify_all();
        if (worker_.joinable()) worker_.join();

        {
            std::lock_guard<std::mutex> lock(file_mutex_);
            ClosePendingFileLocked();
        }

        s_active = NULL;
        if (camera_manager_inited_) {
            DjiCameraManager_DeInit();
            camera_manager_inited_ = false;
        }
        USER_LOG_INFO("[H30T][download] service stopped");
    }

    bool RequestStartShoot() { return Enqueue(Command::kStartShoot); }
    bool RequestStopShoot() { return Enqueue(Command::kStopShoot); }
    bool RequestDownloadOnce() { return Enqueue(Command::kDownloadOnce); }
    bool RequestStatus() { return Enqueue(Command::kStatus); }

    H30tDownloadState State() const { return state_.load(); }

    H30tDownloadStats Stats() const
    {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        return stats_;
    }

    bool SetShootIntervalMs(int milliseconds)
    {
        if (milliseconds < 2000 || milliseconds > 600000) return false;
        shoot_interval_ms_.store(milliseconds);
        Enqueue(Command::kRestartShoot);
        USER_LOG_INFO("[H30T][download] shoot interval set to %d ms", milliseconds);
        return true;
    }
    int ShootIntervalMs() const { return shoot_interval_ms_.load(); }

    bool SetCaptureLensMask(int lens_mask)
    {
        if (lens_mask < 0 || lens_mask > 7) return false;
        capture_lens_mask_.store(lens_mask);
        Enqueue(Command::kRestartShoot);
        USER_LOG_INFO("[H30T][download] capture lens mask set to %d", lens_mask);
        return true;
    }
    int CaptureLensMask() const { return capture_lens_mask_.load(); }
    bool SetAutoDownload(bool enabled)
    {
        auto_enabled_.store(enabled);
        command_cv_.notify_one();
        return true;
    }
    bool AutoDownload() const { return auto_enabled_.load(); }

    bool SetVerifyJpeg(bool enabled)
    {
        verify_jpeg_.store(enabled);
        USER_LOG_INFO("[H30T][download] verify jpeg %s", enabled ? "enabled" : "disabled");
        return true;
    }
    bool VerifyJpeg() const { return verify_jpeg_.load(); }

    bool SetAutoResumeShoot(bool enabled)
    {
        auto_resume_shoot_.store(enabled);
        USER_LOG_INFO("[H30T][download] auto resume shoot %s", enabled ? "enabled" : "disabled");
        return true;
    }
    bool AutoResumeShoot() const { return auto_resume_shoot_.load(); }

    bool SetArchiveAfterDownload(bool enabled)
    {
        if (enabled) {
            std::string error;
            if (!EnsureDirectory(config_.archive_dir, &error)) {
                USER_LOG_ERROR("[H30T][download] %s", error.c_str());
                return false;
            }
        }
        archive_after_download_.store(enabled);
        USER_LOG_INFO("[H30T][download] archive after download %s", enabled ? "enabled" : "disabled");
        return true;
    }
    bool ArchiveAfterDownload() const { return archive_after_download_.load(); }

    // PSDK may include image bytes in the start and end events as well.
    T_DjiReturnCode HandleFileData(const T_DjiDownloadFilePacketInfo &packet,
                                   const uint8_t *data, uint16_t dataLen)
    {
        std::lock_guard<std::mutex> lock(file_mutex_);
        const bool starts = packet.downloadFileEvent == DJI_DOWNLOAD_FILE_EVENT_START
            || packet.downloadFileEvent == DJI_DOWNLOAD_FILE_EVENT_START_TRANSFER_END;
        const bool ends = packet.downloadFileEvent == DJI_DOWNLOAD_FILE_EVENT_END
            || packet.downloadFileEvent == DJI_DOWNLOAD_FILE_EVENT_START_TRANSFER_END;
        if (starts) {
                ClosePendingFileLocked();
                if (pending_part_.empty()) {
                    USER_LOG_ERROR("[H30T][download] data callback without pending target");
                    return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
                }
                file_ = std::fopen(pending_part_.c_str(), "wb");
                if (file_ == NULL) {
                    USER_LOG_ERROR("[H30T][download] open partial file failed: %s",
                                   pending_part_.c_str());
                    return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
                }
                if (packet.fileSize != 0) expected_bytes_ = packet.fileSize;
                received_ = 0;
                transfer_started_ = true;
                transfer_complete_ = false;
        }
        if (dataLen > 0) {
            if (file_ == NULL || data == NULL || std::fwrite(data, 1, dataLen, file_) != dataLen) {
                USER_LOG_ERROR("[H30T][download] write partial file failed");
                return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
            }
            received_ += dataLen;
        }
        if (ends) {
                ClosePendingFileLocked();
                transfer_complete_ = transfer_started_ && expected_bytes_ > 0
                    && received_ == expected_bytes_;
                transfer_started_ = false;
        }
        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    }

    static T_DjiReturnCode FileDataCallback(T_DjiDownloadFilePacketInfo packetInfo,
                                            const uint8_t *data, uint16_t dataLen)
    {
        Impl *impl = s_active;
        if (impl == NULL) return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
        return impl->HandleFileData(packetInfo, data, dataLen);
    }

private:
    enum class Command { kStartShoot, kStopShoot, kDownloadOnce, kRestartShoot, kStatus };

    bool Enqueue(Command command)
    {
        {
            std::lock_guard<std::mutex> lock(command_mutex_);
            if (shutting_down_.load()) return false;
            commands_.push_back(command);
        }
        command_cv_.notify_one();
        return true;
    }

    void Ack(const std::string &status, const std::string &message)
    {
        Ack(kAckAction, status, message);
    }

    void Ack(const std::string &action, const std::string &status,
             const std::string &message)
    {
        if (!ack_) return;
        ack_(action, status, message);
    }

    void WorkerLoop()
    {
        std::chrono::steady_clock::time_point next_download =
            std::chrono::steady_clock::now() + std::chrono::milliseconds(config_.download_interval_ms);
        for (;;) {
            Command command;
            bool has_command = false;
            {
                std::unique_lock<std::mutex> lock(command_mutex_);
                command_cv_.wait_for(lock, std::chrono::milliseconds(500), [this] {
                    return !commands_.empty() || shutting_down_.load();
                });
                if (commands_.empty()) {
                    if (shutting_down_.load()) break;
                } else {
                    command = commands_.front();
                    commands_.pop_front();
                    has_command = true;
                }
            }
            if (shutting_down_.load()) break;
            if (has_command) {
                Execute(command);
                if (command == Command::kDownloadOnce || command == Command::kStartShoot) {
                    next_download = std::chrono::steady_clock::now()
                        + std::chrono::milliseconds(config_.download_interval_ms);
                }
            } else if (auto_enabled_.load() && std::chrono::steady_clock::now() >= next_download) {
                DoDownloadOnce();
                next_download = std::chrono::steady_clock::now()
                    + std::chrono::milliseconds(config_.download_interval_ms);
            }
        }

        // 退出前恢复相机到安全状态
        if (shooting_) {
            std::string error;
            LeaveShootMode(&error);
        }
        ReleaseRights();
        std::lock_guard<std::mutex> lock(file_mutex_);
        ClosePendingFileLocked();
    }

    void Execute(Command command)
    {
        switch (command) {
            case Command::kStartShoot: DoStartShoot(); break;
            case Command::kStopShoot: DoStopShoot(); break;
            case Command::kDownloadOnce: DoDownloadOnce(); break;
            case Command::kRestartShoot:
                if (shooting_) {
                    std::string error;
                    LeaveShootMode(&error);
                    DoStartShoot();
                }
                break;
            case Command::kStatus: DoStatus(); break;
        }
    }

    // ------------------------------------------------------------------
    // 拍照
    // ------------------------------------------------------------------
    void DoStartShoot()
    {
        if (shooting_) {
            state_.store(H30tDownloadState::kShoot);
            Ack("ok", "interval shooting already active");
            return;
        }
        std::string error;
        if (!CheckCameraStorage(&error) || !EnterShootMode(&error)) {
            state_.store(H30tDownloadState::kIdle);
            Ack("error", error);
            return;
        }
        Ack("ok", "interval shooting started");
    }

    void DoStopShoot()
    {
        std::string error;
        if (!LeaveShootMode(&error)) {
            Ack("error", error);
            return;
        }
        state_.store(H30tDownloadState::kIdle);
        Ack("ok", "shooting stopped");
    }

    bool EnterShootMode(std::string *error)
    {
        const E_DjiMountPosition position = ToMountPosition(config_.mount);

        T_DjiReturnCode rc = DjiCameraManager_SetMode(
            position, DJI_CAMERA_MANAGER_WORK_MODE_SHOOT_PHOTO);
        if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            *error = SdkError("switch to shoot photo mode failed", rc);
            return false;
        }

        const int lens_mask = capture_lens_mask_.load();
        if (lens_mask != 0) {
            T_DjiCameraManagerStreamList streams;
            std::memset(&streams, 0, sizeof(streams));
            const E_DjiCameraManagerStreamSource sources[] = {
                DJI_CAMERA_MANAGER_SOURCE_WIDE_CAM,
                DJI_CAMERA_MANAGER_SOURCE_ZOOM_CAM,
                DJI_CAMERA_MANAGER_SOURCE_IR_CAM
            };
            for (int i = 0; i < 3; ++i) {
                if ((lens_mask & (1 << i)) == 0) continue;
                streams.streamSource[streams.size] = sources[i];
                streams.streamStorage[streams.size] = sources[i];
                ++streams.size;
            }
            rc = DjiCameraManager_SetCaptureRecordingStreams(
                position, DJI_CAMERA_MANAGER_CAPTURE_OR_RECORDING_CAPTURE, &streams);
            if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                *error = SdkError("select capture lenses failed", rc);
                return false;
            }
        }

        rc = DjiCameraManager_SetShootPhotoMode(
            position, DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_INTERVAL);
        if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            *error = SdkError("set interval shoot mode failed", rc);
            return false;
        }

        T_DjiCameraPhotoTimeIntervalSettings interval;
        std::memset(&interval, 0, sizeof(interval));
        interval.captureCount = 255; // 持续拍摄直到 StopShootPhoto
        const int shoot_interval_ms = shoot_interval_ms_.load();
        interval.timeIntervalSeconds =
            static_cast<uint16_t>(shoot_interval_ms / 1000);
        interval.timeIntervalMilliseconds =
            static_cast<uint16_t>(shoot_interval_ms % 1000);
        rc = DjiCameraManager_SetPhotoTimeIntervalSettings(position, interval);
        if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            *error = SdkError("set shoot interval failed", rc);
            return false;
        }

        rc = DjiCameraManager_StartShootPhoto(
            position, DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_INTERVAL);
        if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            *error = SdkError("start interval shooting failed", rc);
            return false;
        }

        shooting_ = true;
        state_.store(H30tDownloadState::kShoot);
        return true;
    }

    bool LeaveShootMode(std::string *error)
    {
        if (!shooting_) return true;
        const T_DjiReturnCode rc =
            DjiCameraManager_StopShootPhoto(ToMountPosition(config_.mount));
        if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_WARN("[H30T][download] %s",
                          SdkError("stop interval shooting failed", rc).c_str());
        }
        shooting_ = false;
        if (state_.load() == H30tDownloadState::kShoot) {
            state_.store(H30tDownloadState::kIdle);
        }
        (void)error;
        return true;
    }

    // ------------------------------------------------------------------
    // 下载批次
    // ------------------------------------------------------------------
    void DoDownloadOnce()
    {
        const H30tDownloadState current = state_.load();
        if (current == H30tDownloadState::kDownload || current == H30tDownloadState::kSwitching) {
            Ack("error", "download already in progress");
            return;
        }

        const bool was_shooting = shooting_;
        std::string error;
        if (was_shooting) LeaveShootMode(&error);

        if (!SwitchToDownloadMode(&error)) {
            state_.store(H30tDownloadState::kFailed);
            RestoreShootMode(was_shooting);
            if (shooting_) state_.store(H30tDownloadState::kShoot);
            Ack("error", error);
            return;
        }
        if (!ObtainRights(&error)) {
            state_.store(H30tDownloadState::kFailed);
            RestoreShootMode(was_shooting);
            if (shooting_) state_.store(H30tDownloadState::kShoot);
            Ack("error", error);
            return;
        }

        state_.store(H30tDownloadState::kDownload);
        const bool listed = DownloadBatch(&error);

        ReleaseRights();
        RestoreShootMode(was_shooting);

        if (!listed) {
            if (!shooting_) state_.store(H30tDownloadState::kFailed);
            Ack("error", error);
            return;
        }
        if (state_.load() == H30tDownloadState::kFailed) {
            Ack("error", "download finished but shooting could not resume");
            return;
        }

        const H30tDownloadStats stats = Stats();
        std::ostringstream message;
        message << "batch finished: downloaded=" << stats.downloaded
                << ", skipped=" << stats.skipped
                << ", failed=" << stats.failed
                << ", last_rate=" << FormatBytes(
                       static_cast<std::uint64_t>(stats.last_rate_bytes_per_sec)) << "/s"
                << ", state=" << H30tDownloadStateName(state_.load());
        Ack("ok", message.str());
    }

    bool SwitchToDownloadMode(std::string *error)
    {
        state_.store(H30tDownloadState::kSwitching);
        const E_DjiMountPosition position = ToMountPosition(config_.mount);

        T_DjiReturnCode rc = DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
        for (int attempt = 0; attempt < 3; ++attempt) {
            rc = DjiCameraManager_SetMode(position, DJI_CAMERA_MANAGER_WORK_MODE_MEDIA_DOWNLOAD);
            if (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) break;
            USER_LOG_WARN("[H30T][download] switch to media download failed (attempt %d)",
                          attempt + 1);
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }
        if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            *error = SdkError("switch to media download mode failed", rc);
            return false;
        }

        if (!callback_registered_) {
            rc = DjiCameraManager_RegDownloadFileDataCallback(position, &Impl::FileDataCallback);
            if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                *error = SdkError("register download data callback failed", rc);
                return false;
            }
            callback_registered_ = true;
        }
        return true;
    }

    bool ObtainRights(std::string *error)
    {
        if (rights_held_) return true;
        const E_DjiMountPosition position = ToMountPosition(config_.mount);

        T_DjiReturnCode rc = DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
        for (int attempt = 0; attempt < 3; ++attempt) {
            rc = DjiCameraManager_ObtainDownloaderRights(position);
            if (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) break;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            *error = SdkError("obtain downloader rights failed", rc);
            return false;
        }
        rights_held_ = true;
        return true;
    }

    void ReleaseRights()
    {
        if (!rights_held_) return;
        const T_DjiReturnCode rc =
            DjiCameraManager_ReleaseDownloaderRights(ToMountPosition(config_.mount));
        if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_WARN("[H30T][download] %s",
                          SdkError("release downloader rights failed", rc).c_str());
        }
        rights_held_ = false;
    }

    void RestoreShootMode(bool was_shooting)
    {
        state_.store(H30tDownloadState::kResume);
        const E_DjiMountPosition position = ToMountPosition(config_.mount);

        T_DjiReturnCode rc = DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
        for (int attempt = 0; attempt < 3; ++attempt) {
            rc = DjiCameraManager_SetMode(position, DJI_CAMERA_MANAGER_WORK_MODE_SHOOT_PHOTO);
            if (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) break;
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }
        if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("[H30T][download] %s",
                           SdkError("restore shoot photo mode failed", rc).c_str());
            state_.store(H30tDownloadState::kFailed);
            return;
        }

        if (was_shooting && auto_resume_shoot_.load()) {
            std::string error;
            if (EnterShootMode(&error)) return;
            USER_LOG_ERROR("[H30T][download] resume shooting failed: %s", error.c_str());
            state_.store(H30tDownloadState::kFailed);
            return;
        }
        state_.store(H30tDownloadState::kIdle);
    }

    bool DownloadBatch(std::string *error)
    {
        std::vector<MediaItem> items;
        if (!CollectMediaItems(&items, error)) return false;

        // 按拍摄时间升序下载，grabber 消费顺序即时间顺序
        std::sort(items.begin(), items.end(), MediaItemLess);

        USER_LOG_INFO("[H30T][download] media list ready, %zu JPEG item(s)", items.size());

        int downloaded_this_batch = 0;
        bool success = true;
        for (std::size_t i = 0; i < items.size(); ++i) {
            const std::string final_name = BuildFinalName(items[i]);
            if (manifest_.IsCompleted(final_name)) {
                std::lock_guard<std::mutex> lock(stats_mutex_);
                ++stats_.skipped;
                continue;
            }
            std::string space_error;
            if (!CheckDiskSpace(&space_error)) {
                *error = space_error;
                success = false;
                break;
            }
            if (!DownloadOne(items[i], final_name)) {
                *error = "one or more JPEG downloads failed";
                success = false;
            } else if (++downloaded_this_batch >= config_.max_files_per_batch) {
                break;
            }
            if (shutting_down_.load()) break;
        }
        return success;
    }

    bool CollectMediaItems(std::vector<MediaItem> *items, std::string *error)
    {
        const E_DjiMountPosition position = ToMountPosition(config_.mount);
        const E_DjiCameraManagerFileListCountPerSlice per_slice =
            config_.slice_count == 120
                ? DJI_CAMERA_MANAGER_FILE_LIST_COUNT_120_PER_SLICE
                : DJI_CAMERA_MANAGER_FILE_LIST_COUNT_60_PER_SLICE;

        std::uint16_t start_index = 0;
        for (;;) {
            T_DjiCameraManagerFileList file_list;
            std::memset(&file_list, 0, sizeof(file_list));

            T_DjiCameraManagerSliceConfig slice;
            std::memset(&slice, 0, sizeof(slice));
            slice.sliceStartIndex = start_index;
            slice.countPerSlice = per_slice;

            const T_DjiReturnCode rc = DjiCameraManager_DownloadFileListBySlices(
                position, slice, &file_list);
            if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                *error = SdkError("download file list failed", rc);
                ReleaseFileList(&file_list);
                return false;
            }

            if (file_list.totalCount > 0 && file_list.fileListInfo == NULL) {
                *error = "camera file list is empty although totalCount > 0";
                return false;
            }

            for (uint16_t i = 0; i < file_list.totalCount; ++i) {
                const T_DjiCameraManagerFileListInfo &info = file_list.fileListInfo[i];
                if (info.type != DJI_CAMERA_FILE_TYPE_JPEG) continue;

                MediaItem item;
                item.name = std::string(info.fileName, strnlen(info.fileName,
                                                                DJI_FILE_NAME_SIZE_MAX));
                item.path = std::string(info.filePath, strnlen(info.filePath,
                                                                DJI_FILE_NAME_SIZE_MAX));
                item.index = info.fileIndex;
                item.size = info.fileSize;
                item.year = info.createTime.year;
                item.month = info.createTime.month;
                item.day = info.createTime.day;
                item.hour = info.createTime.hour;
                item.minute = info.createTime.minute;
                item.second = info.createTime.second;
                items->push_back(item);

                // 实机标定用：确认广角原图对应的子文件类型
                for (uint8_t s = 0;
                     info.subFileListInfo != NULL && s < info.subFileListTotalNum; ++s) {
                    const T_DjiCameraManagerSubFileListInfo &sub = info.subFileListInfo[s];
                    USER_LOG_DEBUG("[H30T][download] file %s sub[%u] type=%d size=%u",
                                   item.name.c_str(), s,
                                   static_cast<int>(sub.type), sub.fileSize);
                }
            }

            const std::uint16_t page_count = file_list.totalCount;
            ReleaseFileList(&file_list);

            if (page_count < static_cast<std::uint16_t>(config_.slice_count)) break;
            if (page_count > std::numeric_limits<std::uint16_t>::max() - start_index) {
                *error = "camera file list exceeds SDK slice index range";
                return false;
            }
            start_index = static_cast<std::uint16_t>(start_index + page_count);
        }
        return true;
    }

    bool DownloadOne(const MediaItem &item, const std::string &final_name)
    {
        const std::string part_path = JoinPath(config_.dl_dir, final_name + ".part");
        const E_DjiMountPosition position = ToMountPosition(config_.mount);
        std::string last_error = "unknown error";

        for (int attempt = 0; attempt <= config_.retry_limit; ++attempt) {
            {
                std::lock_guard<std::mutex> lock(file_mutex_);
                ClosePendingFileLocked();
                pending_part_ = part_path;
                expected_bytes_ = item.size;
                received_ = 0;
                transfer_started_ = false;
                transfer_complete_ = false;
            }
            ::unlink(part_path.c_str());

            const std::chrono::steady_clock::time_point begin =
                std::chrono::steady_clock::now();
            const T_DjiReturnCode rc = DjiCameraManager_DownloadFileByIndex(position, item.index);
            const double elapsed_seconds = std::max(
                0.001, std::chrono::duration_cast<std::chrono::microseconds>(
                           std::chrono::steady_clock::now() - begin).count() / 1e6);

            bool complete = false;
            std::uint64_t received = 0;
            {
                std::lock_guard<std::mutex> lock(file_mutex_);
                ClosePendingFileLocked();
                complete = transfer_complete_;
                received = received_;
            }

            if (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS && complete) {
                if (FinalizeFile(part_path, final_name, received, &last_error)) {
                    std::lock_guard<std::mutex> lock(stats_mutex_);
                    ++stats_.downloaded;
                    stats_.bytes += received;
                    stats_.last_rate_bytes_per_sec = received / elapsed_seconds;
                    USER_LOG_INFO("[H30T][download] %s -> %s (%s, %.2f MB/s)",
                                  item.name.c_str(), final_name.c_str(),
                                  FormatBytes(received).c_str(),
                                  stats_.last_rate_bytes_per_sec / (1024.0 * 1024.0));
                    return true;
                }
                USER_LOG_ERROR("[H30T][download] finalize failed for %s: %s",
                               final_name.c_str(), last_error.c_str());
                ::unlink(part_path.c_str());
                break; // 落盘/元数据问题重试无意义
            }

            last_error = rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS
                ? "incomplete transfer"
                : SdkError("download failed", rc);
            USER_LOG_WARN("[H30T][download] %s attempt %d/%d failed: %s",
                          final_name.c_str(), attempt + 1, config_.retry_limit + 1,
                          last_error.c_str());
            ::unlink(part_path.c_str());

            if (attempt < config_.retry_limit) {
                const int backoff_ms = std::min(1 << attempt, 4) * 1000; // 1/2/4 s
                std::this_thread::sleep_for(std::chrono::milliseconds(backoff_ms));
            }
        }

        {
            std::lock_guard<std::mutex> lock(file_mutex_);
            pending_part_.clear();
        }
        manifest_.Record(final_name, item.size, "failed", config_.retry_limit, 0, last_error);
        {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            ++stats_.failed;
        }
        Ack("error", "download failed: " + final_name + " (" + last_error + ")");
        return false;
    }

    bool FinalizeFile(const std::string &part_path, const std::string &final_name,
                      std::uint64_t received,
                      std::string *error)
    {
        JpegExifSummary summary;
        std::string exif_error;
        std::int64_t epoch_ns = 0;
        bool from_gps = false;

        if (h30t_exif::ReadJpegExif(part_path, &summary, &exif_error)
            && h30t_exif::ComputeEpochNs(summary, &epoch_ns, &from_gps, &exif_error)) {
            USER_LOG_INFO("[H30T][download] %s capture time from %s",
                          final_name.c_str(), from_gps ? "GPS" : "local EXIF");
        } else {
            *error = "original JPEG has no usable EXIF capture time: " + exif_error;
            return false;
        }

        if (verify_jpeg_.load()) {
            if (!h30t_exif::VerifyJpeg(part_path, error)) return false;
        }

        const std::string destination = JoinPath(config_.incoming_dir, final_name);
        if (::rename(part_path.c_str(), destination.c_str()) != 0) {
            // dl/ 与 incoming/ 必须在同一挂载点，否则 rename 不是原子操作
            *error = "atomic rename failed: " + std::string(std::strerror(errno));
            return false;
        }
        if (archive_after_download_.load()) {
            const std::string archive_path = JoinPath(config_.archive_dir, final_name);
            if (::link(destination.c_str(), archive_path.c_str()) != 0 && errno != EEXIST) {
                USER_LOG_WARN("[H30T][download] archive link failed for %s: %s",
                              final_name.c_str(), std::strerror(errno));
            }
        }

        manifest_.Record(final_name, static_cast<std::uint32_t>(received), "done", 0,
                         epoch_ns, "");
        return true;
    }

    bool CheckDiskSpace(std::string *error)
    {
        std::uint64_t free_bytes = 0;
        if (!FreeBytes(config_.dl_dir, &free_bytes)) return true; // 查询失败不阻断下载

        if (free_bytes < config_.min_free_bytes) {
            *error = "local disk free space below threshold: " + FormatBytes(free_bytes);
            USER_LOG_ERROR("[H30T][download] %s", error->c_str());
            return false;
        }
        return true;
    }

    bool CheckCameraStorage(std::string *error)
    {
        T_DjiCameraManagerStorageInfo info;
        std::memset(&info, 0, sizeof(info));
        const T_DjiReturnCode rc =
            DjiCameraManager_GetStorageInfo(ToMountPosition(config_.mount), &info);
        if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            *error = SdkError("read camera storage info failed", rc);
            return false;
        }
        if (static_cast<std::uint64_t>(info.remainCapacity) < config_.min_camera_mb) {
            std::ostringstream message;
            message << "camera SD card nearly full: remain=" << info.remainCapacity
                    << " MB, threshold=" << config_.min_camera_mb << " MB";
            *error = message.str();
            return false;
        }
        return true;
    }

    void DoStatus()
    {
        const H30tDownloadStats stats = Stats();
        std::uint64_t free_bytes = 0;
        const bool has_free = FreeBytes(config_.dl_dir, &free_bytes);

        std::ostringstream message;
        message << "state=" << H30tDownloadStateName(state_.load())
                << ", downloaded=" << stats.downloaded
                << ", skipped=" << stats.skipped
                << ", failed=" << stats.failed
                << ", bytes=" << FormatBytes(stats.bytes)
                << ", last_rate=" << FormatBytes(
                       static_cast<std::uint64_t>(stats.last_rate_bytes_per_sec)) << "/s"
                << ", manifest=" << manifest_.Size()
                << ", free=" << (has_free ? FormatBytes(free_bytes) : "unknown");
        Ack("ok", message.str());
    }

    void ClosePendingFileLocked()
    {
        if (file_ == NULL) return;
        std::fflush(file_);
        fsync(fileno(file_));
        std::fclose(file_);
        file_ = NULL;
    }

private:
    H30tDownloadConfig config_;
    H30tDownloadAckCallback ack_;

    DownloadManifest manifest_;
    std::atomic<H30tDownloadState> state_;

    std::thread worker_;
    std::mutex command_mutex_;
    std::condition_variable command_cv_;
    std::deque<Command> commands_;
    std::atomic<bool> shutting_down_;

    std::mutex file_mutex_;
    FILE *file_;
    std::string pending_part_;
    std::uint32_t expected_bytes_;
    std::uint64_t received_;
    bool transfer_started_;
    bool transfer_complete_;

    mutable std::mutex stats_mutex_;
    H30tDownloadStats stats_;

    bool camera_manager_inited_;
    bool callback_registered_;
    bool rights_held_;
    bool shooting_;
    std::atomic<bool> auto_enabled_;

    // 可由 Pilot 2 自定义控件在运行期修改的参数
    std::atomic<int> shoot_interval_ms_;
    std::atomic<int> capture_lens_mask_;
    std::atomic<bool> verify_jpeg_;
    std::atomic<bool> auto_resume_shoot_;
    std::atomic<bool> archive_after_download_;
};

H30tDownloader::Impl *H30tDownloader::Impl::s_active = NULL;

H30tDownloader::H30tDownloader(const H30tDownloadAckCallback &ack)
    : impl_(new Impl(ack))
{
}

H30tDownloader::~H30tDownloader()
{
    impl_->Shutdown();
}

bool H30tDownloader::Start(const H30tDownloadConfig &config, std::string *error)
{
    return impl_->Start(config, error);
}

void H30tDownloader::Shutdown()
{
    impl_->Shutdown();
}

bool H30tDownloader::RequestStartShoot() { return impl_->RequestStartShoot(); }
bool H30tDownloader::RequestStopShoot() { return impl_->RequestStopShoot(); }
bool H30tDownloader::RequestDownloadOnce() { return impl_->RequestDownloadOnce(); }
bool H30tDownloader::RequestStatus() { return impl_->RequestStatus(); }

H30tDownloadState H30tDownloader::State() const { return impl_->State(); }
H30tDownloadStats H30tDownloader::Stats() const { return impl_->Stats(); }

bool H30tDownloader::SetShootIntervalMs(int milliseconds)
{
    return impl_->SetShootIntervalMs(milliseconds);
}
int H30tDownloader::ShootIntervalMs() const { return impl_->ShootIntervalMs(); }

bool H30tDownloader::SetCaptureLensMask(int lens_mask)
{
    return impl_->SetCaptureLensMask(lens_mask);
}
int H30tDownloader::CaptureLensMask() const { return impl_->CaptureLensMask(); }
bool H30tDownloader::SetAutoDownload(bool enabled) { return impl_->SetAutoDownload(enabled); }
bool H30tDownloader::AutoDownload() const { return impl_->AutoDownload(); }

bool H30tDownloader::SetVerifyJpeg(bool enabled) { return impl_->SetVerifyJpeg(enabled); }
bool H30tDownloader::VerifyJpeg() const { return impl_->VerifyJpeg(); }

bool H30tDownloader::SetAutoResumeShoot(bool enabled)
{
    return impl_->SetAutoResumeShoot(enabled);
}
bool H30tDownloader::AutoResumeShoot() const { return impl_->AutoResumeShoot(); }

bool H30tDownloader::SetArchiveAfterDownload(bool enabled)
{
    return impl_->SetArchiveAfterDownload(enabled);
}
bool H30tDownloader::ArchiveAfterDownload() const { return impl_->ArchiveAfterDownload(); }
