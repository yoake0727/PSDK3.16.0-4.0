#ifndef H30T_DOWNLOAD_MANIFEST_HPP
#define H30T_DOWNLOAD_MANIFEST_HPP

#include <cstdint>
#include <map>
#include <mutex>
#include <string>

// 下载台账（manifest.jsonl）
// 每行一条记录，以“最终文件名”为幂等键，不使用列表索引。
class DownloadManifest {
public:
    explicit DownloadManifest(const std::string &path = std::string());

    // 设置台账路径，必须在 Load 之前调用
    void SetPath(const std::string &path);

    // 读取已有台账，用于进程重启后的断点续传
    bool Load(std::string *error);

    bool IsCompleted(const std::string &name) const;
    bool HasEntry(const std::string &name) const;

    // 追加一条记录；status 取 downloading / done / failed
    bool Record(const std::string &name,
                std::uint32_t size,
                const std::string &status,
                int retry,
                std::int64_t exif_ns,
                const std::string &message);

    std::size_t Size() const;

private:
    struct Entry {
        std::string status;
        std::int64_t exif_ns;

        Entry() : exif_ns(0) {}
    };

    bool AppendLine(const std::string &line);

    std::string path_;
    mutable std::mutex mutex_;
    std::map<std::string, Entry> entries_;
};

#endif // H30T_DOWNLOAD_MANIFEST_HPP
