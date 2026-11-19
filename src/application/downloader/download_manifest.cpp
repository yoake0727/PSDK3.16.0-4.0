#include "download_manifest.hpp"

#include "3rdparty/json.hpp"
#include "dji_logger.h"

#include <cstdio>
#include <ctime>
#include <unistd.h>
#include <vector>

using json = nlohmann::json;

DownloadManifest::DownloadManifest(const std::string &path)
    : path_(path)
{
}

void DownloadManifest::SetPath(const std::string &path)
{
    std::lock_guard<std::mutex> lock(mutex_);
    path_ = path;
}

bool DownloadManifest::Load(std::string *error)
{
    std::lock_guard<std::mutex> lock(mutex_);
    entries_.clear();

    FILE *fp = std::fopen(path_.c_str(), "r");
    if (fp == NULL) {
        if (error) *error = "manifest not found: " + path_;
        return false;
    }

    char buffer[8192];
    std::size_t line_no = 0;
    std::size_t broken = 0;
    while (std::fgets(buffer, sizeof(buffer), fp) != NULL) {
        ++line_no;
        std::string line(buffer);
        while (!line.empty() && (line[line.size() - 1] == '\n' || line[line.size() - 1] == '\r')) {
            line.erase(line.size() - 1);
        }
        if (line.empty()) continue;

        json record;
        try {
            record = json::parse(line);
        } catch (...) {
            ++broken;
            continue;
        }
        if (!record.contains("name") || !record["name"].is_string()) {
            ++broken;
            continue;
        }
        Entry entry;
        if (record.contains("status") && record["status"].is_string()) {
            entry.status = record["status"].get<std::string>();
        }
        if (record.contains("exif_ns") && record["exif_ns"].is_number()) {
            entry.exif_ns = record["exif_ns"].get<std::int64_t>();
        }
        entries_[record["name"].get<std::string>()] = entry;
    }
    std::fclose(fp);

    if (broken > 0) {
        USER_LOG_WARN("[H30T][download] manifest has %zu broken line(s) in %s",
                      broken, path_.c_str());
    }
    USER_LOG_INFO("[H30T][download] manifest loaded, %zu entry(s)", entries_.size());
    return true;
}

bool DownloadManifest::IsCompleted(const std::string &name) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    const std::map<std::string, Entry>::const_iterator it = entries_.find(name);
    return it != entries_.end() && it->second.status == "done";
}

bool DownloadManifest::HasEntry(const std::string &name) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return entries_.find(name) != entries_.end();
}

std::size_t DownloadManifest::Size() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return entries_.size();
}

bool DownloadManifest::Record(const std::string &name,
                             std::uint32_t size,
                             const std::string &status,
                             int retry,
                             std::int64_t exif_ns,
                             const std::string &message)
{
    std::lock_guard<std::mutex> lock(mutex_);

    json record;
    record["name"] = name;
    record["size"] = size;
    record["status"] = status;
    record["retry"] = retry;
    record["exif_ns"] = exif_ns;
    if (!message.empty()) record["message"] = message;
    record["ts"] = static_cast<long long>(time(NULL));

    if (!AppendLine(record.dump())) return false;

    Entry &entry = entries_[name];
    entry.status = status;
    entry.exif_ns = exif_ns;
    return true;
}

bool DownloadManifest::AppendLine(const std::string &line)
{
    // 先写文件再更新内存，保证崩溃后以磁盘记录为准
    FILE *fp = std::fopen(path_.c_str(), "a");
    if (fp == NULL) {
        USER_LOG_ERROR("[H30T][download] open manifest failed: %s", path_.c_str());
        return false;
    }
    const bool written = std::fwrite(line.data(), 1, line.size(), fp) == line.size()
        && std::fputc('\n', fp) != EOF;
    std::fflush(fp);
    if (written && fsync(fileno(fp)) != 0) {
        USER_LOG_WARN("[H30T][download] fsync manifest failed: %s", path_.c_str());
    }
    std::fclose(fp);
    return written;
}
