#ifndef H30T_DOWNLOADER_HPP
#define H30T_DOWNLOADER_HPP

#include "download_types.hpp"

#include <memory>
#include <string>

// H30T 原图下载服务
//   - 独立 worker 线程执行 PSDK 相机 API，不占用 PSDK 主循环/心跳线程
//   - 控件开启拍照及周期下载后，分批下载并逐张原子发布原始 JPG
//   - 保留相机原始 EXIF/XMP，由 OpenREALM 读取拍摄时间和位置
class H30tDownloader {
public:
    explicit H30tDownloader(const H30tDownloadAckCallback &ack);
    ~H30tDownloader();

    // 创建目录、加载台账、清理残留 .part 并启动 worker；不会自动拍照
    bool Start(const H30tDownloadConfig &config, std::string *error);
    void Shutdown();

    // 异步命令（由 worker 串行执行，接口本身不阻塞）
    bool RequestStartShoot();     // 进入 SHOOT
    bool RequestStopShoot();      // 停止拍照，回到 IDLE
    bool RequestDownloadOnce();   // 执行一次完整的下载批次
    bool RequestStatus();         // 上报当前状态与统计

    H30tDownloadState State() const;
    H30tDownloadStats Stats() const;

    // 运行期参数（Pilot 2 自定义控件实时修改，worker 下一次执行时生效）
    bool SetShootIntervalMs(int milliseconds);   // 允许范围 [2000, 600000]
    int ShootIntervalMs() const;
    bool SetCaptureLensMask(int lens_mask);      // 0=current setting, 1=wide, 2=zoom, 4=IR
    int CaptureLensMask() const;
    bool SetAutoDownload(bool enabled);
    bool AutoDownload() const;
    bool SetVerifyJpeg(bool enabled);
    bool VerifyJpeg() const;
    bool SetAutoResumeShoot(bool enabled);
    bool AutoResumeShoot() const;
    bool SetArchiveAfterDownload(bool enabled);
    bool ArchiveAfterDownload() const;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

#endif // H30T_DOWNLOADER_HPP
