#ifndef H30T_WIDGET_HPP
#define H30T_WIDGET_HPP

#include "dji_typedef.h"
#include "dji_widget.h"

#include <atomic>
#include <cstddef>
#include <string>
#include <thread>

class H30tDownloader;

// H30T 下载参数自定义控件
//   - 将下载相关参数暴露到 DJI Pilot 2 主界面/配置界面
//   - 用户操作控件时把参数写入 H30tDownloader，URL 命令仍可用
//   - 通过浮窗每秒刷新一次状态，便于现场确认
class H30tWidget {
public:
    H30tWidget(H30tDownloader *downloader, std::string resource_dir);
    ~H30tWidget();

    // 仅初始化控件模块并注册 UI 配置，不依赖下载器，可重复调用
    // 必须在 DjiCameraManager_Init() 之前调用：PSDK 下载模块初始化时会校验控件
    // UI 配置，未注册会导致下载数据初始化失败（stat = 0x000000E3）
    static bool InitUiConfig(const std::string &resource_dir, std::string *error);
    static bool UiConfigReady();

    // Handler 表必须在 DjiCore_ApplicationStart() 前注册，并且在整个进程内有效。
    // 注册完成后，控件回调会先经过静态安全入口，再转发到当前活动实例。
    static bool RegisterHandlers(std::string *error);

    // 初始化控件模块、注册 UI 配置与控件处理器，并启动浮窗刷新线程
    bool Start(std::string *error);
    void Stop();

    bool Started() const { return started_.load(); }

private:
    static void SetActiveWidget(H30tWidget *widget);

    static T_DjiReturnCode SetWidgetValueCallback(E_DjiWidgetType widgetType,
                                                  uint32_t index, int32_t value,
                                                  void *userData);
    static T_DjiReturnCode GetWidgetValueCallback(E_DjiWidgetType widgetType,
                                                  uint32_t index, int32_t *value,
                                                  void *userData);

    T_DjiReturnCode HandleSetWidgetValue(E_DjiWidgetType widgetType,
                                         uint32_t index, int32_t value);
    T_DjiReturnCode HandleGetWidgetValue(E_DjiWidgetType widgetType,
                                         uint32_t index, int32_t *value);

    void FloatingWindowLoop();
    void PublishFloatingMessage();

    H30tDownloader *downloader_;
    std::string resource_dir_;
    std::atomic<bool> started_;
    std::atomic<bool> stopping_;
    std::thread floating_thread_;
};

#endif // H30T_WIDGET_HPP
