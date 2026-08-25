#include "gimbal_controller.hpp"

#include <cstdio>

GimbalController::~GimbalController()
{
    Deinitialize();
}

bool GimbalController::Initialize()
{
    if (initialized_) return true;
    const T_DjiReturnCode rc = DjiGimbalManager_Init();
    initialized_ = (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
    return initialized_;
}

void GimbalController::Deinitialize()
{
    if (!initialized_) return;
    (void)DjiGimbalManager_Deinit();
    initialized_ = false;
}

T_DjiReturnCode GimbalController::Rotate(E_DjiMountPosition mount,
                                         E_DjiGimbalRotationMode mode,
                                         float pitch, float roll, float yaw,
                                         double time, std::string &error) const
{
    if (!initialized_) {
        error = "gimbal manager not initialized";
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }
    if (time <= 0.0 || time > 60.0) {
        error = "time must be greater than 0 and no more than 60 seconds";
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }
    if (pitch == 0.0F && roll == 0.0F && yaw == 0.0F) {
        error = "at least one angle must be non-zero";
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }
    T_DjiGimbalManagerRotation rotation = {};
    rotation.rotationMode = mode;
    rotation.pitch = pitch;
    rotation.roll = roll;
    rotation.yaw = yaw;
    rotation.time = time;
    return DjiGimbalManager_Rotate(mount, rotation);
}
