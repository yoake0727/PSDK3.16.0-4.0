#include "h30t_liveview_session.hpp"

#include "h30t_config.hpp"
#include "h30t_rgb_stream_pipeline.hpp"

#include "dji_camera_manager.h"
#include "dji_liveview.h"
#include "dji_logger.h"

#include <iomanip>
#include <chrono>
#include <atomic>
#include <mutex>
#include <sstream>
#include <thread>

namespace {

const E_DjiLiveViewCameraSource kImageSource = DJI_LIVEVIEW_CAMERA_SOURCE_DEFAULT;

std::mutex g_dispatch_mutex;
// 回调必须共享管线生命周期，Detach 后才允许销毁对象。
std::shared_ptr<H30tRgbStreamPipeline> g_rgb_pipeline;
E_DjiLiveViewCameraPosition g_camera_position = DJI_LIVEVIEW_CAMERA_POSITION_NO_1;
std::atomic<bool> g_rgb_format_logged(false);

void InfraredImageCallback(E_DjiLiveViewCameraPosition position,
                           const uint8_t *data, uint32_t length,
                           T_DjiLiveviewImageInfo image_info)
{
    try {
        if (!g_rgb_format_logged.exchange(true)) {
            USER_LOG_INFO("H30T RGB image format: %ux%u, pixFmt=%d, frameId=%u",
                          static_cast<unsigned int>(image_info.width),
                          static_cast<unsigned int>(image_info.height),
                          static_cast<int>(image_info.pixFmt),
                          static_cast<unsigned int>(image_info.frameId));
        }
        std::shared_ptr<H30tRgbStreamPipeline> pipeline;
        { std::lock_guard<std::mutex> lock(g_dispatch_mutex);
          if (position != g_camera_position || image_info.pixFmt != PIXFMT_RGB_PACKED) return;
          pipeline = g_rgb_pipeline; }
        if (pipeline) pipeline->PushRgb(data, length, image_info.width, image_info.height);
    } catch (...) { USER_LOG_ERROR("Unhandled exception in H30T RGB callback."); }
}

void Attach(const std::shared_ptr<H30tRgbStreamPipeline> &rgb,
            E_DjiLiveViewCameraPosition position)
{
    std::lock_guard<std::mutex> lock(g_dispatch_mutex);
    g_camera_position = position; g_rgb_pipeline = rgb;
}

void Detach()
{
    std::lock_guard<std::mutex> lock(g_dispatch_mutex);
    g_rgb_pipeline.reset();
}

bool ConvertMount(int input, E_DjiMountPosition &mount,
                  E_DjiLiveViewCameraPosition &camera)
{
    switch (input) {
        case 1: mount = DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1;
                camera = DJI_LIVEVIEW_CAMERA_POSITION_NO_1; return true;
        case 2: mount = DJI_MOUNT_POSITION_PAYLOAD_PORT_NO2;
                camera = DJI_LIVEVIEW_CAMERA_POSITION_NO_2; return true;
        case 3: mount = DJI_MOUNT_POSITION_PAYLOAD_PORT_NO3;
                camera = DJI_LIVEVIEW_CAMERA_POSITION_NO_3; return true;
        default: return false;
    }
}

std::string SdkError(const char *message, T_DjiReturnCode code)
{
    std::ostringstream out;
    out << message << " (SDK code=0x" << std::hex
        << static_cast<unsigned long long>(code) << ")";
    return out.str();
}

} // namespace

class H30tLiveviewSession::Impl {
public:
    Impl() : camera_manager(false), liveview(false), attached(false),
             zoom_started(false), infrared_started(false),
             mount(DJI_MOUNT_POSITION_UNKNOWN),
             camera(DJI_LIVEVIEW_CAMERA_POSITION_NO_1),
             active_source(DJI_CAMERA_MANAGER_SOURCE_DEFAULT_CAM) {}

    bool camera_manager;
    bool liveview;
    bool attached;
    bool zoom_started;
    bool infrared_started;
    E_DjiMountPosition mount;
    E_DjiLiveViewCameraPosition camera;
    std::shared_ptr<H30tRgbStreamPipeline> infrared;
    H30tStreamPipelineConfig pipeline_config;
    E_DjiCameraManagerStreamSource active_source;
};

H30tLiveviewSession::H30tLiveviewSession() : impl_(new Impl) {}
H30tLiveviewSession::~H30tLiveviewSession() { Stop(); }

bool H30tLiveviewSession::Start(int mount, const H30tRtspConfig &config,
                                std::string &error)
{
    Stop();
    if (!ConvertMount(mount, impl_->mount, impl_->camera)) {
        error = "invalid mount position"; return false;
    }
    T_DjiReturnCode code = DjiCameraManager_Init();
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        error = SdkError("camera manager initialization failed", code); return false;
    }
    impl_->camera_manager = true;
    bool connected = false;
    code = DjiCameraManager_GetCameraConnectStatus(impl_->mount, &connected);
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS || !connected) {
        error = SdkError("H30T is not connected", code); Stop(); return false;
    }
    E_DjiCameraType type = DJI_CAMERA_TYPE_UNKNOWN;
    code = DjiCameraManager_GetCameraType(impl_->mount, &type);
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS || type != DJI_CAMERA_TYPE_H30T) {
        error = SdkError("camera at requested mount is not H30T", code); Stop(); return false;
    }
    code = DjiLiveview_Init();
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        error = SdkError("liveview initialization failed", code); Stop(); return false;
    }
    impl_->liveview = true;

    H30tStreamPipelineConfig infrared_config;
    infrared_config.rtsp_url = config.rtsp_url;
    infrared_config.rtsp_transport = config.transport;
    infrared_config.max_queue_bytes = config.max_queue_bytes;
    impl_->pipeline_config = infrared_config;
    const E_DjiCameraManagerStreamSource default_source = DJI_CAMERA_MANAGER_SOURCE_IR_CAM;
    code = DjiCameraManager_SetStreamSource(impl_->mount, default_source);
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        error = SdkError("H30T infrared source selection failed", code);
        Stop();
        return false;
    }
    bool source_ready = false;
    for (int i = 0; i < 10; ++i) {
        T_DjiCameraCurrentCameraStatus status = {};
        if (DjiCameraManager_GetCurrentCameraStatus(impl_->mount, &status)
            == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS
            && static_cast<E_DjiCameraManagerStreamSource>(status.liveview_source_stream) == default_source) {
            source_ready = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (!source_ready) {
        error = "H30T infrared source selection timed out";
        Stop();
        return false;
    }
    impl_->active_source = default_source;
    impl_->infrared.reset(new H30tRgbStreamPipeline);
    if (!impl_->infrared->Start(infrared_config)) {
        error = "RGB RTSP pipeline initialization failed"; Stop(); return false;
    }
    Attach(impl_->infrared, impl_->camera);
    impl_->attached = true;
    g_rgb_format_logged.store(false);
    code = DjiLiveview_StartImageStream(impl_->camera, kImageSource, PIXFMT_RGB_PACKED, InfraredImageCallback);
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        error = SdkError("H30T RGB image subscription failed", code); Stop(); return false;
    }
    impl_->infrared_started = true;
    USER_LOG_INFO("H30T switchable RGB RTSP target: %s",
                  h30t_config::RedactRtspUrl(config.rtsp_url).c_str());
    return true;
}

void H30tLiveviewSession::ServiceIntraframeRequests()
{
}

H30tStreamStatus H30tLiveviewSession::Status() const
{ return impl_->infrared ? impl_->infrared->SnapshotStatus() : H30tStreamStatus(); }

bool H30tLiveviewSession::SwitchSource(H30tSource source)
{
    E_DjiCameraManagerStreamSource manager_source;
    switch (source) {
        case H30tSource::kWide: manager_source = DJI_CAMERA_MANAGER_SOURCE_WIDE_CAM; break;
        case H30tSource::kZoom: manager_source = DJI_CAMERA_MANAGER_SOURCE_ZOOM_CAM; break;
        case H30tSource::kInfrared: manager_source = DJI_CAMERA_MANAGER_SOURCE_IR_CAM; break;
        default: return false;
    }
    const T_DjiReturnCode code = DjiCameraManager_SetStreamSource(impl_->mount, manager_source);
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) return false;
    for (int i = 0; i < 10; ++i) {
        T_DjiCameraCurrentCameraStatus status = {};
        if (DjiCameraManager_GetCurrentCameraStatus(impl_->mount, &status)
            == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS
            && static_cast<E_DjiCameraManagerStreamSource>(status.liveview_source_stream) == manager_source) {
            impl_->active_source = manager_source;
            USER_LOG_INFO("H30T source switched without restarting RGB/RTSP pipeline: %d", static_cast<int>(manager_source));
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return false;
}

void H30tLiveviewSession::Stop()
{
    if (impl_->attached) { Detach(); impl_->attached = false; }
    if (impl_->infrared_started) {
        DjiLiveview_StopImageStream(impl_->camera, kImageSource);
        impl_->infrared_started = false;
    }
    if (impl_->infrared) impl_->infrared->Stop();
    impl_->infrared.reset();
    if (impl_->liveview) { DjiLiveview_Deinit(); impl_->liveview = false; }
    if (impl_->camera_manager) { DjiCameraManager_DeInit(); impl_->camera_manager = false; }
}
