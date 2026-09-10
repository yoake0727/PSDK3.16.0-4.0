#include "minio_uploader.hpp"
#include "app_config.hpp"
#include <miniocpp/args.h>
#include <miniocpp/client.h>
#include <miniocpp/providers.h>
#include <condition_variable>
#include <mutex>
#include <sstream>
#include <thread>

/**
 * 单个上传任务。
 *   jpeg      : JPEG 字节流
 *   name      : 对象文件名（不含目录前缀）
 *   locations : 定位结果 JSON 字符串，上传完成后随回调返回
 *   infrared  : true=红外（INF_images），false=可见光（DroneFiles）
 *   done      : 完成回调，参数 (上传结果, locations)
 */
struct MinioUploader::Task {
    std::vector<uint8_t> jpeg;
    std::string name, locations;
    bool infrared;
    Completion done;
};

/**
 * 构造：分配队列、互斥锁、条件变量。
 * 用 void* 存储是为了在头文件里不暴露 STL 类型（PImpl 手写版）。
 */
MinioUploader::MinioUploader() {
    queue_     = new std::vector<Task>();
    mutex_     = new std::mutex();
    condition_ = new std::condition_variable();
}

/**
 * 析构：先 Stop() 停线程，再释放资源。
 */
MinioUploader::~MinioUploader() {
    Stop();
    delete static_cast<std::vector<Task>*>(queue_);
    delete static_cast<std::mutex*>(mutex_);
    delete static_cast<std::condition_variable*>(condition_);
}

/**
 * 启动上传器。
 *
 * 1. 从 YAML 读 MinIO 配置（地址、桶、目录）
 * 2. 从统一配置读取 access key / secret key
 * 3. 启动工作线程
 *
 * 注意：
 *   - access_key / secret_key 缺失则启动失败
 *   - 与旧版不同：不再拆分 host/port，address_ 原样交给 minio-cpp 的 BaseUrl
 *   - 返回 false 时 e 里带原因
 */
bool MinioUploader::Start(std::string &e) {
    const AppConfig &app=AppConfig::Instance();
    address_=app.String("minio.addr"); bucket_=app.String("minio.bucket");
    visible_folder_=app.String("minio.visible_folder"); infrared_folder_=app.String("minio.infrared_folder");
    access_key_=app.String("minio.access_key"); secret_key_=app.String("minio.secret_key");
    if (access_key_.empty() || secret_key_.empty()) {
        e = "MinIO credentials missing in app_config.yaml";
        return false;
    }

    running_ = true;
    worker_  = new std::thread(&MinioUploader::WorkerLoop, this);
    return true;
}

/**
 * 入队一个上传任务。
 *
 * 返回：
 *   true  : 已入队
 *   false : 未启动 / JPEG 为空 / 队列已满（>= 4）
 *
 * 队列上限 4：防止上游检测过快导致内存积压。
 * 入队成功后 notify_one 唤醒工作线程。
 */
bool MinioUploader::Enqueue(const std::vector<uint8_t> &j,
                            const std::string &n,
                            bool ir,
                            const std::string &l,
                            const Completion &cb) {
    if (!running_ || j.empty()) return false;

    std::lock_guard<std::mutex> g(*static_cast<std::mutex*>(mutex_));
    if (queue_->size() >= 4) return false;

    queue_->push_back(Task{j, n, l, ir, cb});
    static_cast<std::condition_variable*>(condition_)->notify_one();
    return true;
}

/**
 * 工作线程主循环。
 *
 * 1. 等待条件：running_ 为 false 或队列非空
 * 2. 队列空且已停止 → 退出
 * 3. 取出队首任务（锁外执行上传，避免阻塞入队）
 * 4. 调用 Upload，完成后触发回调
 */
void MinioUploader::WorkerLoop() {
    while (true) {
        Task t;
        {
            std::unique_lock<std::mutex> g(*static_cast<std::mutex*>(mutex_));
            static_cast<std::condition_variable*>(condition_)->wait(
                g, [this] { return !running_ || !queue_->empty(); });
            if (!running_ && queue_->empty()) break;

            t = queue_->front();
            queue_->erase(queue_->begin());
        }

        MinioUploadResult r = Upload(t);
        if (t.done) t.done(r, t.locations);
    }
}

/**
 * 实际上传：用 minio-cpp 官方 SDK 做 PUT。
 *
 * 流程：
 *   1. 拼对象路径（folder/name）
 *   2. 构造 BaseUrl + StaticProvider（凭证）
 *   3. 构造 Client
 *   4. 把 JPEG 字节流写进 stringstream
 *   5. 构造 PutObjectArgs（bucket / object / Content-Type / 分片大小）
 *   6. 调 PutObject
 *   7. 成功则填 file_url；失败填 error
 *
 * 关键点（与旧版的区别）：
 *   - 签名由 SDK 内部完成，是标准 AWS SigV4，MinIO 端正常校验
 *   - 不依赖服务端宽松策略，换 S3/OSS 也能用
 *   - 分片大小 16MB：图片远小于此值，实际是一次性上传
 *   - BaseUrl 第三个参数 "us-east-1" 是 region，MinIO 一般不校验
 */
MinioUploadResult MinioUploader::Upload(const Task &t) const {
    MinioUploadResult r;
    r.object_name = (t.infrared ? infrared_folder_ : visible_folder_) + "/" + t.name;

    try {
        // BaseUrl：address_ 形如 "host:port"，secure=false 表示 HTTP
        minio::s3::BaseUrl url(address_, false, "us-east-1");

        // 凭证：access key / secret key
        minio::creds::StaticProvider provider(access_key_, secret_key_);

        // 客户端
        minio::s3::Client client(url, &provider);

        // 把 JPEG 字节流写入 stringstream（SDK 要求 istream）
        std::stringstream stream;
        stream.write(reinterpret_cast<const char*>(t.jpeg.data()), t.jpeg.size());

        // PutObject 参数：流、长度、分片大小（16MB）
        minio::s3::PutObjectArgs args(stream,
                                      static_cast<uint64_t>(t.jpeg.size()),
                                      16 * 1024 * 1024UL);
        args.bucket = bucket_;
        args.object = r.object_name;
        args.headers["Content-Type"] = "image/jpeg";

        // 执行上传
        auto response = client.PutObject(args);
        if (!response) {
            r.error = response.error().String();
            return r;
        }

        r.success  = true;
        r.file_url = "http://" + address_ + "/" + bucket_ + "/" + r.object_name;
    } catch (const std::exception &e) {
        r.error = e.what();
    }
    return r;
}

/**
 * 停止上传器。
 *
 * 1. 置 running_ = false
 * 2. notify_all 唤醒工作线程
 * 3. join 等待线程退出（会先处理完队列里的任务）
 * 4. delete 线程对象
 *
 * 幂等：running_ 已为 false 时直接返回。
 */
void MinioUploader::Stop() {
    if (!running_) return;

    {
        std::lock_guard<std::mutex> g(*static_cast<std::mutex*>(mutex_));
        running_ = false;
    }
    static_cast<std::condition_variable*>(condition_)->notify_all();

    std::thread *t = static_cast<std::thread*>(worker_);
    if (t && t->joinable()) t->join();
    delete t;
    worker_ = nullptr;
}
