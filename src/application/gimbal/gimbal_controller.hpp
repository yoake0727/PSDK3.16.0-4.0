#pragma once

#include <string>
#include "dji_gimbal_manager.h"

class GimbalController {
public:
    ~GimbalController();
    bool Initialize();
    void Deinitialize();
    bool IsInitialized() const { return initialized_; }
    T_DjiReturnCode Rotate(E_DjiMountPosition mount, E_DjiGimbalRotationMode mode,
                           float pitch, float roll, float yaw, double time,
                           std::string &error) const;

private:
    bool initialized_ = false;
};
