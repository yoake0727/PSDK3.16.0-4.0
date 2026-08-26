#include "gimbal_controller.hpp"

#include <cstdio>

GimbalController::~GimbalController()
{
    Deinitialize();
}
// 初始化云台管理器
bool GimbalController::Initialize()
{
    if (initialized_) return true;
    const T_DjiReturnCode rc = DjiGimbalManager_Init();
    initialized_ = (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
    return initialized_;
}
// 反初始化云台管理器
void GimbalController::Deinitialize()
{
    if (!initialized_) return;
    (void)DjiGimbalManager_Deinit();
    initialized_ = false;
}
// 旋转云台到指定角度（相对/绝对）
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
// 一键让云台指向正下方
T_DjiReturnCode GimbalController::PointDownward(E_DjiMountPosition mount,
                                                std::string &error) const
{
    if (!initialized_) {
        error = "gimbal manager not initialized";
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }
    return DjiGimbalManager_Reset(
        mount, DJI_GIMBAL_RESET_MODE_PITCH_DOWNWARD_UPWARD);
}
// 复位云台（多种模式）
T_DjiReturnCode GimbalController::Reset(E_DjiMountPosition mount,
                                        E_DjiGimbalResetMode mode,
                                        std::string &error) const
{
    if (!initialized_) {
        error = "gimbal manager not initialized";
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }
    return DjiGimbalManager_Reset(mount, mode);
}
