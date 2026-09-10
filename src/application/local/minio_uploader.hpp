#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

struct MinioUploadResult {
    bool success = false;
    std::string object_name;
    std::string file_url;
    std::string error;
};

class MinioUploader {
public:
    typedef std::function<void(const MinioUploadResult &, const std::string &)> Completion;
    MinioUploader();
    ~MinioUploader();
    bool Start(std::string &error);
    bool Enqueue(const std::vector<uint8_t> &jpeg, const std::string &objectName,
                 bool infrared, const std::string &locationJson, const Completion &completion);
    void Stop();
private:
    struct Task;
    void WorkerLoop();
    MinioUploadResult Upload(const Task &task) const;
    std::string address_, bucket_, visible_folder_, infrared_folder_, access_key_, secret_key_;
    int port_ = 0;
    int jpeg_quality_ = 0;
    bool running_ = false;
    std::vector<Task> *queue_ = nullptr;
    void *mutex_ = nullptr;
    void *condition_ = nullptr;
    void *worker_ = nullptr;
};
