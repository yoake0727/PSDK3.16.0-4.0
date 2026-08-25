// http_downloader.h
#pragma once

#include <string>

class HttpDownloader {
public:
    // 下载 URL 到本地文件，成功返回 true
    static bool Download(const std::string& url, const std::string& local_path);
};