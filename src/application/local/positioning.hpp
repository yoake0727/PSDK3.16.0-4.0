#pragma once

namespace local_positioning {

struct LeverArm {
    double forward_m = 0.0;
    double right_m = 0.0;
    double down_m = 0.0;
    LeverArm() {}
    LeverArm(double forward, double right, double down)
        : forward_m(forward), right_m(right), down_m(down) {}
};

struct GimbalPosition {
    double latitude_deg;
    double longitude_deg;
    double height_agl_m;
};

struct GeoPosition {
    double latitude_deg;
    double longitude_deg;
};

GimbalPosition computeGimbalPosition(double rtkLatitudeDeg,
                                     double rtkLongitudeDeg,
                                     double rtkHeightAglM,
                                     double yawDeg,
                                     double pitchDeg,
                                     double rollDeg,
                                     const LeverArm &leverArm);

GeoPosition pixelToGpsWithYaw(double cameraLatitudeDeg,
                              double cameraLongitudeDeg,
                              double cameraHeightAglM,
                              double targetHeightAglM,
                              double focalLengthMm,
                              double pixelSizeMm,
                              int imageWidth,
                              int imageHeight,
                              double pixelU,
                              double pixelV,
                              double yawDeg,
                              double pitchDeg = 0.0,
                              double rollDeg = 0.0,
                              double cameraPitchDeg = 0.0);

} // namespace local_positioning
