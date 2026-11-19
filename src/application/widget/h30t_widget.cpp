#include "h30t_widget.hpp"

#include "downloader/h30t_downloader.hpp"
#include "downloader/download_types.hpp"

#include "dji_logger.h"
#include "dji_widget.h"

#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <vector>

#include <unistd.h>

namespace {

// 控件编号：主界面 0-3，配置界面 4-7（编号不能重复）
const uint32_t kWidgetStartShoot = 0;
const uint32_t kWidgetStopShoot = 1;
const uint32_t kWidgetDownloadNow = 2;
const uint32_t kWidgetAutoResumeShoot = 3;
const uint32_t kWidgetShootIntervalSeconds = 4;
const uint32_t kWidgetCaptureLensMask = 5;
const uint32_t kWidgetVerifyJpeg = 6;
const uint32_t kWidgetArchiveAfterDownload = 7;
const uint32_t kWidgetAutoDownload = 8;
const uint32_t kWidgetCount = 9;

const int kShootIntervalMinSeconds = 2;    // 与 Validate 的 2000 ms 对齐
const int kShootIntervalMaxSeconds = 600;
const int kCaptureLensMaskMax = 7;

std::string JoinPath(const std::string &base, const std::string &name)
{
    if (base.empty()) return name;
    if (base[base.size() - 1] == '/') return base + name;
    return base + "/" + name;
}

bool DirectoryExists(const std::string &path)
{
    return ::access(path.c_str(), F_OK) == 0;
}

// 取可执行文件所在目录，用于默认定位 widget_file
std::string ExecutableDirectory()
{
    char buffer[1024];
    const ssize_t length = ::readlink("/proc/self/exe", buffer, sizeof(buffer) - 1);
    if (length <= 0) return std::string();
    buffer[length] = '\0';

    std::string path(buffer);
    const std::size_t slash = path.find_last_of('/');
    if (slash == std::string::npos) return std::string();
    return path.substr(0, slash);
}

// 控件配置文件目录优先级：环境变量 > 可执行文件同级 > 常见部署路径
std::string ResolveWidgetDir()
{
    const char *env_dir = std::getenv("H30T_WIDGET_DIR");
    if (env_dir != NULL && env_dir[0] != '\0') return std::string(env_dir);

    std::vector<std::string> candidates;
    const std::string exe_dir = ExecutableDirectory();
    if (!exe_dir.empty()) {
        candidates.push_back(JoinPath(exe_dir, "widget_file"));
        candidates.push_back(JoinPath(JoinPath(exe_dir, ".."), "widget_file"));
        candidates.push_back(JoinPath(JoinPath(exe_dir, "../.."), "widget_file"));
    }
    // 兼容从项目根目录、安装包根目录直接启动程序的情况。
    candidates.push_back("./widget_file");
    candidates.push_back("../widget_file");
    candidates.push_back("/data/H30T/widget_file");
    candidates.push_back("/data/h30t/widget_file");

    for (std::size_t i = 0; i < candidates.size(); ++i) {
        if (DirectoryExists(JoinPath(candidates[i], "cn_big_screen"))) {
            return candidates[i];
        }
    }
    return candidates.empty() ? std::string("/data/h30t/widget_file") : candidates[0];
}

// 控件 UI 配置只需注册一次，且要早于相机管理器初始化
std::mutex g_ui_config_mutex;
bool g_ui_config_ready = false;

// Handler 表在进程内只注册一次；活动实例由 Start()/Stop() 切换，
// 这样即便下载服务未启动，点击控件也能进入我们的回调并给出明确日志。
std::mutex g_handler_reg_mutex;
bool g_handlers_registered = false;

std::mutex g_active_mutex;
H30tWidget *g_active_widget = nullptr;

int32_t SafeWidgetValue(uint32_t index)
{
    // 无业务实例（下载服务未启动）时回给 Pilot 的安全默认值
    switch (index) {
        case 4:  // 拍照间隔(秒)
            return 5;
        default:
            return 0;  // 按钮=释放、开关=关、镜头掩码=0
    }
}

std::string FormatRate(double bytes_per_second)
{
    char buffer[32];
    if (bytes_per_second >= 1024.0 * 1024.0) {
        std::snprintf(buffer, sizeof(buffer), "%.1fMB/s", bytes_per_second / (1024.0 * 1024.0));
    } else {
        std::snprintf(buffer, sizeof(buffer), "%.0fKB/s", bytes_per_second / 1024.0);
    }
    return std::string(buffer);
}

} // namespace

H30tWidget::H30tWidget(H30tDownloader *downloader, std::string resource_dir)
    : downloader_(downloader),
      resource_dir_(resource_dir.empty() ? ResolveWidgetDir() : resource_dir),
      started_(false),
      stopping_(false)
{
}

H30tWidget::~H30tWidget()
{
    Stop();
}

bool H30tWidget::InitUiConfig(const std::string &resource_dir, std::string *error)
{
    std::lock_guard<std::mutex> lock(g_ui_config_mutex);
    if (g_ui_config_ready) return true;

    const std::string dir = resource_dir.empty() ? ResolveWidgetDir() : resource_dir;
    std::string local_error;
    std::string *err = error ? error : &local_error;

    const std::string en_dir = JoinPath(dir, "en_big_screen");
    const std::string cn_dir = JoinPath(dir, "cn_big_screen");

    // 目录缺失时提前给出明确原因，避免 SDK 侧只返回 0x000000E3
    if (!DirectoryExists(JoinPath(dir, "cn_big_screen")) &&
        !DirectoryExists(JoinPath(dir, "en_big_screen"))) {
        *err = "widget resource dir not found: " + dir;
        USER_LOG_ERROR("[H30T][widget] %s", err->c_str());
        return false;
    }

    T_DjiReturnCode rc = DjiWidget_Init();
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        *err = "DjiWidget_Init failed";
        USER_LOG_ERROR("[H30T][widget] %s, rc=0x%08llX", err->c_str(),
                       static_cast<unsigned long long>(rc));
        return false;
    }

    // 未覆盖的语言/屏幕类型回落到默认配置
    rc = DjiWidget_RegDefaultUiConfigByDirPath(en_dir.c_str());
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        *err = "register default widget ui config failed: " + en_dir;
        USER_LOG_ERROR("[H30T][widget] %s, rc=0x%08llX", err->c_str(),
                       static_cast<unsigned long long>(rc));
        return false;
    }

    rc = DjiWidget_RegUiConfigByDirPath(DJI_MOBILE_APP_LANGUAGE_ENGLISH,
                                        DJI_MOBILE_APP_SCREEN_TYPE_BIG_SCREEN,
                                        en_dir.c_str());
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        *err = "register english widget ui config failed: " + en_dir;
        USER_LOG_ERROR("[H30T][widget] %s, rc=0x%08llX", err->c_str(),
                       static_cast<unsigned long long>(rc));
        return false;
    }

    rc = DjiWidget_RegUiConfigByDirPath(DJI_MOBILE_APP_LANGUAGE_CHINESE,
                                        DJI_MOBILE_APP_SCREEN_TYPE_BIG_SCREEN,
                                        cn_dir.c_str());
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        *err = "register chinese widget ui config failed: " + cn_dir;
        USER_LOG_ERROR("[H30T][widget] %s, rc=0x%08llX", err->c_str(),
                       static_cast<unsigned long long>(rc));
        return false;
    }

    g_ui_config_ready = true;
    USER_LOG_INFO("[H30T][widget] widget ui config registered, resource dir=%s", dir.c_str());
    return true;
}

bool H30tWidget::UiConfigReady()
{
    std::lock_guard<std::mutex> lock(g_ui_config_mutex);
    return g_ui_config_ready;
}

void H30tWidget::SetActiveWidget(H30tWidget *widget)
{
    std::lock_guard<std::mutex> lock(g_active_mutex);
    g_active_widget = widget;
}

bool H30tWidget::RegisterHandlers(std::string *error)
{
    std::lock_guard<std::mutex> lock(g_handler_reg_mutex);
    if (g_handlers_registered) return true;

    std::string local_error;
    std::string *err = error ? error : &local_error;

    static T_DjiWidgetHandlerListItem handlers[kWidgetCount];
    std::memset(handlers, 0, sizeof(handlers));

    handlers[0].widgetIndex = kWidgetStartShoot;
    handlers[0].widgetType = DJI_WIDGET_TYPE_BUTTON;
    handlers[1].widgetIndex = kWidgetStopShoot;
    handlers[1].widgetType = DJI_WIDGET_TYPE_BUTTON;
    handlers[2].widgetIndex = kWidgetDownloadNow;
    handlers[2].widgetType = DJI_WIDGET_TYPE_BUTTON;
    handlers[3].widgetIndex = kWidgetAutoResumeShoot;
    handlers[3].widgetType = DJI_WIDGET_TYPE_SWITCH;
    handlers[4].widgetIndex = kWidgetShootIntervalSeconds;
    handlers[4].widgetType = DJI_WIDGET_TYPE_INT_INPUT_BOX;
    handlers[5].widgetIndex = kWidgetCaptureLensMask;
    handlers[5].widgetType = DJI_WIDGET_TYPE_INT_INPUT_BOX;
    handlers[6].widgetIndex = kWidgetVerifyJpeg;
    handlers[6].widgetType = DJI_WIDGET_TYPE_SWITCH;
    handlers[7].widgetIndex = kWidgetArchiveAfterDownload;
    handlers[7].widgetType = DJI_WIDGET_TYPE_SWITCH;
    handlers[8].widgetIndex = kWidgetAutoDownload;
    handlers[8].widgetType = DJI_WIDGET_TYPE_SWITCH;

    for (uint32_t i = 0; i < kWidgetCount; ++i) {
        handlers[i].SetWidgetValue = &H30tWidget::SetWidgetValueCallback;
        handlers[i].GetWidgetValue = &H30tWidget::GetWidgetValueCallback;
        handlers[i].userData = NULL;
    }

    const T_DjiReturnCode rc = DjiWidget_RegHandlerList(handlers, kWidgetCount);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        *err = "DjiWidget_RegHandlerList failed";
        USER_LOG_ERROR("[H30T][widget] DjiWidget_RegHandlerList failed, rc=0x%08llX",
                       static_cast<unsigned long long>(rc));
        return false;
    }

    g_handlers_registered = true;
    USER_LOG_INFO("[H30T][widget] widget handlers registered");
    return true;
}

bool H30tWidget::Start(std::string *error)
{
    if (started_.load()) return true;
    if (downloader_ == NULL) {
        if (error) *error = "downloader is not available";
        return false;
    }

    std::string local_error;
    std::string *err = error ? error : &local_error;

    // UI 配置通常已在 DjiCameraManager_Init() 之前注册，这里做幂等兜底
    if (!InitUiConfig(resource_dir_, err)) {
        return false;
    }

    if (!RegisterHandlers(err)) {
        return false;
    }

    SetActiveWidget(this);

    stopping_.store(false);
    started_.store(true);

    // 浮窗常驻消息会占用 Pilot 2 的悬浮区，默认关闭，避免主界面控件被挤掉；
    // 需要实时状态时设置 H30T_WIDGET_FLOATING=1 开启。
    const char *floating = std::getenv("H30T_WIDGET_FLOATING");
    if (floating != NULL && std::strcmp(floating, "1") == 0) {
        floating_thread_ = std::thread(&H30tWidget::FloatingWindowLoop, this);
    } else {
        USER_LOG_INFO("[H30T][widget] floating window disabled (set H30T_WIDGET_FLOATING=1 to enable)");
    }

    USER_LOG_INFO("[H30T][widget] custom widget started, resource dir=%s",
                  resource_dir_.c_str());
    return true;
}

void H30tWidget::Stop()
{
    if (!started_.exchange(false)) return;
    SetActiveWidget(NULL);
    stopping_.store(true);
    if (floating_thread_.joinable()) floating_thread_.join();
    USER_LOG_INFO("[H30T][widget] custom widget stopped");
}

T_DjiReturnCode H30tWidget::SetWidgetValueCallback(E_DjiWidgetType widgetType,
                                                  uint32_t index, int32_t value,
                                                  void *userData)
{
    (void)userData;
    std::lock_guard<std::mutex> lock(g_active_mutex);
    H30tWidget *self = g_active_widget;

    if (self == NULL) {
        // 下载服务未启动时控件仍会显示，点击后在这里留下明确原因
        if (widgetType == DJI_WIDGET_TYPE_BUTTON &&
            value == DJI_WIDGET_BUTTON_STATE_PRESS_DOWN) {
            USER_LOG_WARN("[H30T][widget] widget %u pressed but download service is not running "
                          "(check H30T_DL_ENABLE=1)", index);
        }
        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    }

    return self->HandleSetWidgetValue(widgetType, index, value);
}

T_DjiReturnCode H30tWidget::GetWidgetValueCallback(E_DjiWidgetType widgetType,
                                                  uint32_t index, int32_t *value,
                                                  void *userData)
{
    (void)userData;
    if (value == NULL) return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;

    std::lock_guard<std::mutex> lock(g_active_mutex);
    H30tWidget *self = g_active_widget;

    if (self == NULL) {
        *value = SafeWidgetValue(index);
        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    }
    return self->HandleGetWidgetValue(widgetType, index, value);
}

T_DjiReturnCode H30tWidget::HandleSetWidgetValue(E_DjiWidgetType widgetType,
                                                uint32_t index, int32_t value)
{
    if (downloader_ == NULL) return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;

    switch (index) {
        case kWidgetStartShoot:
            if (widgetType != DJI_WIDGET_TYPE_BUTTON) break;
            if (value != DJI_WIDGET_BUTTON_STATE_PRESS_DOWN) {
                return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
            }
            return downloader_->RequestStartShoot()
                ? DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS
                : DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;

        case kWidgetStopShoot:
            if (widgetType != DJI_WIDGET_TYPE_BUTTON) break;
            if (value != DJI_WIDGET_BUTTON_STATE_PRESS_DOWN) {
                return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
            }
            return downloader_->RequestStopShoot()
                ? DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS
                : DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;

        case kWidgetDownloadNow:
            if (widgetType != DJI_WIDGET_TYPE_BUTTON) break;
            if (value != DJI_WIDGET_BUTTON_STATE_PRESS_DOWN) {
                return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
            }
            return downloader_->RequestDownloadOnce()
                ? DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS
                : DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;

        case kWidgetAutoResumeShoot:
            if (widgetType != DJI_WIDGET_TYPE_SWITCH) break;
            downloader_->SetAutoResumeShoot(value == DJI_WIDGET_SWITCH_STATE_ON);
            return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;

        case kWidgetShootIntervalSeconds:
            if (widgetType != DJI_WIDGET_TYPE_INT_INPUT_BOX) break;
            if (value < kShootIntervalMinSeconds || value > kShootIntervalMaxSeconds) {
                USER_LOG_WARN("[H30T][widget] shoot interval %d s out of range [%d, %d]",
                              value, kShootIntervalMinSeconds, kShootIntervalMaxSeconds);
                return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
            }
            return downloader_->SetShootIntervalMs(value * 1000)
                ? DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS
                : DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;

        case kWidgetCaptureLensMask:
            if (widgetType != DJI_WIDGET_TYPE_INT_INPUT_BOX) break;
            if (value < 0 || value > kCaptureLensMaskMax) {
                USER_LOG_WARN("[H30T][widget] lens mask %d out of range [0, %d]",
                              value, kCaptureLensMaskMax);
                return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
            }
            return downloader_->SetCaptureLensMask(value)
                ? DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS
                : DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;

        case kWidgetAutoDownload:
            if (widgetType != DJI_WIDGET_TYPE_SWITCH) break;
            downloader_->SetAutoDownload(value == DJI_WIDGET_SWITCH_STATE_ON);
            return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;

        case kWidgetVerifyJpeg:
            if (widgetType != DJI_WIDGET_TYPE_SWITCH) break;
            downloader_->SetVerifyJpeg(value == DJI_WIDGET_SWITCH_STATE_ON);
            return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;

        case kWidgetArchiveAfterDownload:
            if (widgetType != DJI_WIDGET_TYPE_SWITCH) break;
            return downloader_->SetArchiveAfterDownload(value == DJI_WIDGET_SWITCH_STATE_ON)
                ? DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS
                : DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;

        default:
            return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }

    // 控件类型与配置不一致，直接拒绝，避免误写参数
    USER_LOG_WARN("[H30T][widget] widget %u type mismatch: %d", index,
                  static_cast<int>(widgetType));
    return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
}

T_DjiReturnCode H30tWidget::HandleGetWidgetValue(E_DjiWidgetType widgetType,
                                                uint32_t index, int32_t *value)
{
    if (value == NULL) return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    if (downloader_ == NULL) {
        *value = SafeWidgetValue(index);
        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    }

    switch (index) {
        case kWidgetStartShoot:
        case kWidgetStopShoot:
        case kWidgetDownloadNow:
            if (widgetType != DJI_WIDGET_TYPE_BUTTON) break;
            *value = DJI_WIDGET_BUTTON_STATE_RELEASE_UP;
            return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;

        case kWidgetAutoResumeShoot:
            if (widgetType != DJI_WIDGET_TYPE_SWITCH) break;
            *value = downloader_->AutoResumeShoot()
                ? DJI_WIDGET_SWITCH_STATE_ON : DJI_WIDGET_SWITCH_STATE_OFF;
            return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;

        case kWidgetShootIntervalSeconds:
            if (widgetType != DJI_WIDGET_TYPE_INT_INPUT_BOX) break;
            *value = downloader_->ShootIntervalMs() / 1000;
            return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;

        case kWidgetCaptureLensMask:
            if (widgetType != DJI_WIDGET_TYPE_INT_INPUT_BOX) break;
            *value = downloader_->CaptureLensMask();
            return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;

        case kWidgetAutoDownload:
            if (widgetType != DJI_WIDGET_TYPE_SWITCH) break;
            *value = downloader_->AutoDownload()
                ? DJI_WIDGET_SWITCH_STATE_ON : DJI_WIDGET_SWITCH_STATE_OFF;
            return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;

        case kWidgetVerifyJpeg:
            if (widgetType != DJI_WIDGET_TYPE_SWITCH) break;
            *value = downloader_->VerifyJpeg()
                ? DJI_WIDGET_SWITCH_STATE_ON : DJI_WIDGET_SWITCH_STATE_OFF;
            return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;

        case kWidgetArchiveAfterDownload:
            if (widgetType != DJI_WIDGET_TYPE_SWITCH) break;
            *value = downloader_->ArchiveAfterDownload()
                ? DJI_WIDGET_SWITCH_STATE_ON : DJI_WIDGET_SWITCH_STATE_OFF;
            return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;

        default:
            return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
}

void H30tWidget::FloatingWindowLoop()
{
    while (!stopping_.load()) {
        PublishFloatingMessage();
        for (int i = 0; i < 10 && !stopping_.load(); ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
}

void H30tWidget::PublishFloatingMessage()
{
    if (downloader_ == NULL) return;

    const H30tDownloadState state = downloader_->State();
    const H30tDownloadStats stats = downloader_->Stats();

    char message[DJI_WIDGET_FLOATING_WINDOW_MSG_MAX_LEN + 1];
    std::snprintf(message, sizeof(message),
                  "H30T DL %s | %ds | ok:%u skip:%u fail:%u | %s",
                  H30tDownloadStateName(state),
                  downloader_->ShootIntervalMs() / 1000,
                  stats.downloaded, stats.skipped, stats.failed,
                  FormatRate(stats.last_rate_bytes_per_sec).c_str());

    // 内容不变时不重复推送，避免 1Hz 刷新干扰 Pilot 2 的控件显示
    static std::string last_message;
    if (last_message == message) return;
    last_message = message;

    const T_DjiReturnCode rc = DjiWidgetFloatingWindow_ShowMessage(message);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_DEBUG("[H30T][widget] floating window message failed, rc=0x%08llX",
                       static_cast<unsigned long long>(rc));
    }
}
