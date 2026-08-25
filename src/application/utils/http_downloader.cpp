#include "http_downloader.hpp"
#include "dji_logger.h"
#include "dji_platform.h"

#include <cstdio>
#include <cstring>
#include <string>
#include <regex>
#include <netdb.h>       // getaddrinfo, freeaddrinfo
#include <sys/socket.h>  // AF_INET, SOCK_STREAM
#include <unistd.h>      // close (备用)

// 辅助函数：解析 URL (声明)
static bool ParseUrl(const std::string& url, std::string& host, int& port, std::string& path);

bool HttpDownloader::Download(const std::string& url, const std::string& local_path) {
    // 获取 PSDK 网络处理器
    T_DjiSocketHandler* socketHandler = DjiPlatform_GetSocketHandler();
    if (!socketHandler) {
        USER_LOG_ERROR("Socket handler not available");
        return false;
    }

    // 1. 解析 URL
    std::string host;
    int port;
    std::string path;
    if (!ParseUrl(url, host, port, path)) {
        return false;
    }
    USER_LOG_INFO("HTTP download: %s -> %s (host=%s:%d, path=%s)",
                  url.c_str(), local_path.c_str(), host.c_str(), port, path.c_str());

    // 2. 创建 socket
    int sock = ::socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        USER_LOG_ERROR("Failed to create socket");
        return false;
    }

    // 3. 解析域名并连接
    struct addrinfo hints = {0};
    struct addrinfo* result = nullptr;
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    char port_str[8];
    snprintf(port_str, sizeof(port_str), "%d", port);

    if (getaddrinfo(host.c_str(), port_str, &hints, &result) != 0) {
        USER_LOG_ERROR("Failed to resolve host: %s", host.c_str());
        ::close(sock);
        return false;
    }

    int ret = -1;
    for (struct addrinfo* rp = result; rp != nullptr; rp = rp->ai_next) {
        ret = ::connect(sock, rp->ai_addr, rp->ai_addrlen);
        if (ret == 0) break;
    }
    freeaddrinfo(result);

    if (ret != 0) {
        USER_LOG_ERROR("Failed to connect to %s:%d", host.c_str(), port);
        ::close(sock);
        return false;
    }

    // 4. 发送 HTTP GET 请求
    std::string request = "GET " + path + " HTTP/1.1\r\n" +
                          "Host: " + host + "\r\n" +
                          "Connection: close\r\n" +
                          "\r\n";
    int sent = ::send(sock, request.c_str(), request.size(), 0);
    if (sent != (int)request.size()) {
        USER_LOG_ERROR("Failed to send HTTP request");
        ::close(sock);
        return false;
    }

    // 5. 接收响应头，定位空行
    std::string header;
    char buf[4096];
    bool header_end = false;
    size_t content_length = 0;
    bool chunked = false;
    while (!header_end) {
        int n = ::recv(sock, (uint8_t*)buf, sizeof(buf) - 1, 0);
        if (n <= 0) break;
        buf[n] = '\0';
        header.append(buf, n);
        size_t pos = header.find("\r\n\r\n");
        if (pos != std::string::npos) {
            // 解析 Content-Length
            std::string header_part = header.substr(0, pos);
            size_t cl_pos = header_part.find("Content-Length:");
            if (cl_pos != std::string::npos) {
                content_length = std::stoull(header_part.substr(cl_pos + 16));
            }
            if (header_part.find("Transfer-Encoding: chunked") != std::string::npos) {
                chunked = true;
            }
            // 保留剩余数据作为 body 开始
            std::string body = header.substr(pos + 4);
            header = body;  // 将剩余数据移到 header 变量中，稍后写入文件
            header_end = true;
        }
    }

    if (!header_end) {
        USER_LOG_ERROR("Failed to parse HTTP response header");
        ::close(sock);
        return false;
    }

    // 6. 打开本地文件
    FILE* fp = fopen(local_path.c_str(), "wb");
    if (!fp) {
        USER_LOG_ERROR("Failed to open local file for writing: %s", local_path.c_str());
        ::close(sock);
        return false;
    }

    // 写入已读取的 body 数据
    if (!header.empty()) {
        fwrite(header.c_str(), 1, header.size(), fp);
    }

    // 继续接收剩余数据
    size_t total_written = header.size();
    while (true) {
        int n = ::recv(sock, buf, sizeof(buf) - 1, 0);
        if (n <= 0) break;
        fwrite(buf, 1, n, fp);
        total_written += n;
    }

    fclose(fp);
    ::close(sock);

    USER_LOG_INFO("HTTP download completed: %zu bytes written to %s", total_written, local_path.c_str());
    return true;
}

// 辅助函数实现：解析 URL
static bool ParseUrl(const std::string& url, std::string& host, int& port, std::string& path) {
    // 格式: http://host[:port]/path
    std::regex re(R"(http://([^:/]+)(?::(\d+))?(/.*)?)");
    std::smatch match;
    if (!std::regex_match(url, match, re)) {
        USER_LOG_ERROR("Invalid URL format: %s", url.c_str());
        return false;
    }
    host = match[1].str();
    if (match[2].matched) {
        port = std::stoi(match[2].str());
    } else {
        port = 80;
    }
    if (match[3].matched) {
        path = match[3].str();
    } else {
        path = "/";
    }
    return true;
}