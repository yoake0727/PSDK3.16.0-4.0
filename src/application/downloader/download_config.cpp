#include "download_config.hpp"

#include <cerrno>
#include <climits>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include <sys/stat.h>
#include <unistd.h>

namespace {

// 逐级创建目录；成功返回 true
bool EnsureWritableDir(const std::string &dir)
{
    if (dir.empty()) return false;

    std::string current;
    std::size_t pos = 0;
    if (dir[0] == '/') {
        current = "/";
        pos = 1;
    }
    while (pos <= dir.size()) {
        const std::size_t slash = dir.find('/', pos);
        const std::string part = dir.substr(
            pos, slash == std::string::npos ? std::string::npos : slash - pos);
        if (!part.empty()) {
            if (current.size() > 1 && current[current.size() - 1] != '/') current += '/';
            current += part;
            if (mkdir(current.c_str(), 0755) != 0 && errno != EEXIST) return false;
        }
        if (slash == std::string::npos) break;
        pos = slash + 1;
    }
    return access(dir.c_str(), W_OK | X_OK) == 0;
}

std::string ExecutableDir()
{
    char buffer[PATH_MAX];
    const ssize_t length = readlink("/proc/self/exe", buffer, sizeof(buffer) - 1);
    if (length <= 0) return std::string();
    buffer[length] = '\0';
    const std::string path(buffer);
    const std::size_t slash = path.find_last_of('/');
    return (slash == std::string::npos) ? std::string() : path.substr(0, slash);
}

// 默认根目录可能因权限不可写（如 /data 由 root 独占），
// 这里按可写性依次探测，避免整个下载服务因建目录失败而退出。
std::string ResolveDefaultRoot()
{
    std::vector<std::string> candidates;
    const std::string exe_dir = ExecutableDir();
    if (!exe_dir.empty()) candidates.push_back(exe_dir + "/h30t_data");

    const char *home = std::getenv("HOME");
    if (home != NULL && home[0] != '\0') {
        candidates.push_back(std::string(home) + "/h30t_data");
    }
    candidates.push_back("/data/local/tmp/h30t");
    candidates.push_back("/tmp/h30t");

    for (std::size_t i = 0; i < candidates.size(); ++i) {
        if (EnsureWritableDir(candidates[i])) return candidates[i];
    }
    return "/tmp/h30t";
}

} // namespace

namespace {

std::string EnvString(const char *name, const std::string &fallback)
{
    const char *value = std::getenv(name);
    return (value != NULL && value[0] != '\0') ? std::string(value) : fallback;
}

int EnvInt(const char *name, int fallback)
{
    const char *value = std::getenv(name);
    if (value == NULL || value[0] == '\0') return fallback;
    errno = 0;
    char *end = NULL;
    const long parsed = std::strtol(value, &end, 10);
    if (errno != 0 || end == value || *end != '\0' || parsed < INT_MIN || parsed > INT_MAX) {
        return fallback;
    }
    return static_cast<int>(parsed);
}

unsigned long long EnvULL(const char *name, unsigned long long fallback)
{
    const char *value = std::getenv(name);
    if (value == NULL || value[0] == '\0') return fallback;
    errno = 0;
    char *end = NULL;
    const unsigned long long parsed = std::strtoull(value, &end, 10);
    if (errno != 0 || end == value || *end != '\0') return fallback;
    return parsed;
}

bool EnvBool(const char *name, bool fallback)
{
    const char *value = std::getenv(name);
    if (value == NULL || value[0] == '\0') return fallback;
    return std::strcmp(value, "1") == 0 || std::strcmp(value, "true") == 0;
}

} // namespace

namespace h30t_download_config {

std::string JoinPath(const std::string &base, const std::string &name)
{
    if (base.empty()) return name;
    if (name.empty()) return base;
    if (base[base.size() - 1] == '/') return base + name;
    return base + "/" + name;
}

H30tDownloadConfig LoadFromEnvironment()
{
    H30tDownloadConfig config;

    // 未显式指定根目录/子目录时，选一个当前进程可写的根目录，
    // 避免默认的 /data/h30t 因权限不足导致整个下载服务启动失败
    const char *root_env = std::getenv("H30T_DL_ROOT");
    if (root_env != NULL && root_env[0] != '\0') {
        config.root_dir = root_env;
    } else if (std::getenv("H30T_DL_DIR") == NULL
               || std::getenv("H30T_DL_INCOMING") == NULL
               || std::getenv("H30T_DL_ARCHIVE") == NULL
               || std::getenv("H30T_DL_MANIFEST") == NULL) {
        config.root_dir = ResolveDefaultRoot();
    }
    config.dl_dir = EnvString("H30T_DL_DIR", JoinPath(config.root_dir, "dl"));
    config.incoming_dir = EnvString("H30T_DL_INCOMING", JoinPath(config.root_dir, "incoming"));
    config.archive_dir = EnvString("H30T_DL_ARCHIVE", JoinPath(config.root_dir, "archive"));
    config.manifest_path = EnvString("H30T_DL_MANIFEST", JoinPath(config.root_dir, "manifest.jsonl"));

    config.mount = EnvInt("H30T_DL_MOUNT", config.mount);
    config.shoot_interval_ms = EnvInt("H30T_DL_SHOOT_INTERVAL_MS", config.shoot_interval_ms);
    config.download_interval_ms = EnvInt("H30T_DL_DOWNLOAD_INTERVAL_MS", config.download_interval_ms);
    config.max_files_per_batch = EnvInt("H30T_DL_MAX_FILES_PER_BATCH", config.max_files_per_batch);
    config.slice_count = EnvInt("H30T_DL_SLICE_COUNT", config.slice_count);
    config.capture_lens_mask = EnvInt("H30T_DL_CAPTURE_LENS_MASK", config.capture_lens_mask);
    config.retry_limit = EnvInt("H30T_DL_RETRY", config.retry_limit);

    config.min_free_bytes = static_cast<std::uint64_t>(
        EnvULL("H30T_DL_MIN_FREE_MB", config.min_free_bytes / (1024ULL * 1024ULL)))
        * 1024ULL * 1024ULL;
    config.min_camera_mb = static_cast<std::uint64_t>(
        EnvULL("H30T_DL_MIN_CAMERA_MB", config.min_camera_mb));

    config.verify_jpeg = EnvBool("H30T_DL_VERIFY", config.verify_jpeg);
    config.archive_after_download = EnvBool("H30T_DL_ARCHIVE_AFTER", config.archive_after_download);
    config.auto_resume_shoot = EnvBool("H30T_DL_AUTO_RESUME_SHOOT", config.auto_resume_shoot);
    config.init_camera_manager = EnvBool("H30T_DL_INIT_CAMERA", config.init_camera_manager);

    return config;
}

bool Validate(const H30tDownloadConfig &config, std::string *error)
{
    const auto fail = [&](const char *reason) {
        if (error) *error = reason;
        return false;
    };

    if (config.root_dir.empty()) return fail("root dir is empty");
    if (config.dl_dir.empty()) return fail("download dir is empty");
    if (config.incoming_dir.empty()) return fail("incoming dir is empty");
    if (config.manifest_path.empty()) return fail("manifest path is empty");
    if (config.dl_dir == config.incoming_dir) {
        return fail("dl dir must differ from incoming dir (list scan would pick up .part files)");
    }
    if (config.dl_dir.size() > config.incoming_dir.size()
        && config.dl_dir.compare(0, config.incoming_dir.size(), config.incoming_dir) == 0
        && config.dl_dir[config.incoming_dir.size()] == '/') {
        return fail("dl dir must not be nested inside incoming dir");
    }
    if (config.mount < 1 || config.mount > 3) return fail("mount must be 1, 2 or 3");
    if (config.shoot_interval_ms < 2000 || config.shoot_interval_ms > 600000) {
        return fail("shoot interval must be within [2000, 600000] ms");
    }
    if (config.download_interval_ms < 5000 || config.download_interval_ms > 3600000) {
        return fail("download interval must be within [5000, 3600000] ms");
    }
    if (config.max_files_per_batch < 1 || config.max_files_per_batch > 120) {
        return fail("max files per batch must be within [1, 120]");
    }
    if (config.slice_count != 60 && config.slice_count != 120) {
        return fail("slice count must be 60 or 120");
    }
    if (config.capture_lens_mask < 0 || config.capture_lens_mask > 7) {
        return fail("capture lens mask must be within [0, 7]");
    }
    if (config.retry_limit < 0 || config.retry_limit > 10) {
        return fail("retry limit must be within [0, 10]");
    }
    if (config.min_free_bytes == 0) return fail("min free bytes must be positive");
    return true;
}

} // namespace h30t_download_config
