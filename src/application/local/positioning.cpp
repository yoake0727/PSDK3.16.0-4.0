#include "positioning.hpp"

#include <cmath>
#include <stdexcept>

namespace local_positioning {
namespace {
// WGS84 赤道半径（米）。局部切平面近似用它把 ENU/NED 偏移换算成经纬度增量
constexpr double kEarthRadiusM = 6378137.0;
constexpr double kPi = 3.14159265358979323846;
// 角度 <-> 弧度
double degToRad(double value) { return value * kPi / 180.0; }
double radToDeg(double value) { return value * 180.0 / kPi; }
}

/**
 * 计算吊舱（相机）光心的地理位姿。
 *
 * 机体坐标系约定（与 pixelToGpsWithYaw 保持一致）：
 *   X 轴：机头方向（前）
 *   Y 轴：右翼方向（右）
 *   Z 轴：垂直机身向下
 *
 * 姿态角约定（与无人机遥测一致）：
 *   yaw   ：航向角，0°=正北，顺时针（向右转）为正
 *   pitch ：俯仰角，机头向上为正，向下为负
 *   roll  ：横滚角，右翼向下为正，左翼向下为负
 *
 * 参数：
 *   rtkLatitudeDeg / rtkLongitudeDeg : RTK 天线经纬度（十进制度）
 *   rtkHeightAglM                    : RTK 天线离地高度（米）
 *   yawDeg / pitchDeg / rollDeg      : 无人机姿态角（度）
 *   leverArm                         : 机体系安装偏移 {forward_m, right_m, down_m}（米）
 *
 * 返回：
 *   GimbalPosition {latitude_deg, longitude_deg, height_agl_m}
 *
 * 异常：
 *   纬度接近极点时，经度局部切平面换算失效，抛 std::domain_error
 */
GimbalPosition computeGimbalPosition(double rtkLatitudeDeg,
                                     double rtkLongitudeDeg,
                                     double rtkHeightAglM,
                                     double yawDeg,
                                     double pitchDeg,
                                     double rollDeg,
                                     const LeverArm &leverArm)
{
    // 角度转弧度，并预计算三角函数，避免重复调用
    const double yaw = degToRad(yawDeg);
    const double pitch = degToRad(pitchDeg);
    const double roll = degToRad(rollDeg);
    const double cy = std::cos(yaw), sy = std::sin(yaw);
    const double cp = std::cos(pitch), sp = std::sin(pitch);
    const double cr = std::cos(roll), sr = std::sin(roll);

    const double north = (cy * cp) * leverArm.forward_m
        + (cy * sp * sr - sy * cr) * leverArm.right_m
        + (cy * sp * cr + sy * sr) * leverArm.down_m;
    const double east = (sy * cp) * leverArm.forward_m
        + (sy * sp * sr + cy * cr) * leverArm.right_m
        + (sy * sp * cr - cy * sr) * leverArm.down_m;
    const double down = (-sp) * leverArm.forward_m
        + (cp * sr) * leverArm.right_m
        + (cp * cr) * leverArm.down_m;

    const double latitudeRad = degToRad(rtkLatitudeDeg);
    const double cosLatitude = std::cos(latitudeRad);
    if (std::abs(cosLatitude) < 1e-12) {
        throw std::domain_error("纬度接近极点，无法进行经度局部切平面换算");
    }
    return {rtkLatitudeDeg + radToDeg(north / kEarthRadiusM),
            rtkLongitudeDeg + radToDeg(east / (kEarthRadiusM * cosLatitude)),
            rtkHeightAglM - down};
}

/**
 * 将图像像素坐标投影到地面，得到目标的地理坐标（WGS84）。
 *
 * 假设：
 *   目标位于离地高度 targetHeightAglM 的水平面上，相机光心高度为 cameraHeightAglM。
 *   相机光轴与机体系存在固定安装倾角 cameraPitchDeg（朝机头方向为正）。
 *
 * 坐标系与角度约定同 computeGimbalPosition：
 *   yaw  ：0°=正北，顺时针为正
 *   pitch：机头向上为正
 *   roll ：右翼向下为正
 *
 * 参数：
 *   cameraLatitudeDeg / cameraLongitudeDeg / cameraHeightAglM
 *       相机光心的经纬度与离地高度（一般由 computeGimbalPosition 得到）
 *   targetHeightAglM
 *       目标所在水平面的离地高度（地面目标传 0）
 *   focalLengthMm / pixelSizeMm
 *       相机焦距（mm）与单像元物理尺寸（mm/pixel）
 *   imageWidth / imageHeight
 *       图像分辨率（像素）
 *   pixelU / pixelV
 *       目标在图像中的像素坐标（原点左上角，u 向右，v 向下）
 *   yawDeg / pitchDeg / rollDeg
 *       无人机姿态角（度）
 *   cameraPitchDeg
 *       相机相对机体的安装俯仰角（度，朝机头方向倾斜为正；0 表示相机垂直向下）
 *
 * 返回：
 *   GeoPosition {latitude_deg, longitude_deg}
 *
 * 异常：
 *   内参/尺寸非法 -> std::invalid_argument
 *   俯仰角接近水平，无法与地面相交 -> std::domain_error
 *   纬度接近极点 -> std::domain_error
 */
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
                              double pitchDeg,
                              double rollDeg,
                              double cameraPitchDeg)
{
    if (focalLengthMm <= 0.0 || pixelSizeMm <= 0.0 || imageWidth <= 0 || imageHeight <= 0) {
        throw std::invalid_argument("相机内参或图像尺寸无效");
    }
    // 相机与目标的高度差（米），用于计算 GSD
    const double heightDiff = cameraHeightAglM - targetHeightAglM;
     // 角度转弧度
    const double pitch = degToRad(pitchDeg);
    const double roll = degToRad(rollDeg);
    const double yaw = degToRad(yawDeg);
    const double cameraPitch = degToRad(cameraPitchDeg);
    // 综合俯仰角 = 无人机俯仰 + 相机安装俯仰
    // 它决定光轴相对水平面的夹角，用于斜距换算
    const double totalPitch = pitch + cameraPitch;
    const double cosPitch = std::cos(totalPitch);
    if (std::abs(cosPitch) < 1e-12) {
        throw std::domain_error("相机俯视角接近水平，无法进行地面投影");
    }
    // 焦距换算到像素单位：f_pix = f_mm / pixel_size_mm
    const double focalLengthPix = focalLengthMm / pixelSizeMm;
    // 像素偏移（以图像中心为原点）
    //   deltaU：右为正
    //   deltaV：下为正
    const double deltaU = pixelU - static_cast<double>(imageWidth) / 2.0;
    const double deltaV = pixelV - static_cast<double>(imageHeight) / 2.0;
    // 地面采样距离 GSD（米/像素）：
    //   斜距 = heightDiff / cos(totalPitch)
    //   GSD  = 斜距 / f_pix
    // 含义：图像上 1 像素对应地面上的多少米
    const double gsd = (heightDiff / cosPitch) / focalLengthPix;
    // 像素偏移 -> 机体系水平偏移
    //   offsetXBody：机体系右向（图像右对应机体右）
    //   offsetYBody：机体系前向
    //     图像下方对应机尾方向，所以取负号；
    //     再减去相机安装角带来的投影中心偏移 heightDiff * tan(cameraPitch)
    const double offsetXBody = deltaU * gsd;
    const double offsetYBody = -deltaV * gsd - heightDiff * std::tan(cameraPitch);
    // 考虑 roll 对水平偏移的影响：
    //   横滚使机体系 x 轴（右）不再水平，其在水平面上的投影为 cos(roll)；
    //   同时 x_body 会通过 roll 在纵向产生一个附加分量，
    //   该附加分量再经 totalPitch 投影到地面，得到 sin(roll) * tan(totalPitch)。
    const double groundOffsetX = offsetXBody * std::cos(roll);
    const double groundOffsetY = offsetYBody + offsetXBody * std::sin(roll) * std::tan(totalPitch);
    // 机体系水平偏移 -> 地理系 NED（北、东）
    //   deltaNorth = 前向分量 * cos(yaw) - 右向分量 * sin(yaw)
    //   deltaEast  = 前向分量 * sin(yaw) + 右向分量 * cos(yaw)
    const double deltaNorth = groundOffsetY * std::cos(yaw) - groundOffsetX * std::sin(yaw);
    const double deltaEast = groundOffsetY * std::sin(yaw) + groundOffsetX * std::cos(yaw);
    // 局部切平面近似：EN 偏移 -> 经纬度增量
    const double latitudeRad = degToRad(cameraLatitudeDeg);
    if (std::abs(std::cos(latitudeRad)) < 1e-12) {
        throw std::domain_error("纬度接近极点，无法进行经度局部切平面换算");
    }
    return {cameraLatitudeDeg + radToDeg(deltaNorth / kEarthRadiusM),
            cameraLongitudeDeg + radToDeg(deltaEast / (kEarthRadiusM * std::cos(latitudeRad)))};
}

} // namespace local_positioning
