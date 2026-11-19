#ifndef H30T_EXIF_WRITER_HPP
#define H30T_EXIF_WRITER_HPP

#include <cstdint>
#include <string>

// 从 JPEG 中读到的建图关键元数据摘要
struct JpegExifSummary {
    bool has_datetime_original;   // Exif.Photo.DateTimeOriginal
    int year, month, day, hour, minute, second;

    bool has_gps_date;            // Exif.GPSInfo.GPSDateStamp (UTC)
    int gps_year, gps_month, gps_day;

    bool has_gps_time;            // Exif.GPSInfo.GPSTimeStamp (UTC)
    int gps_hour, gps_minute;
    double gps_second;

    bool has_gimbal_yaw;          // Xmp.drone-dji.GimbalYawDegree
    double gimbal_yaw;

    JpegExifSummary();
};

// 与 OpenREALM 解耦的元数据后处理工具：
//   - ReadJpegExif        : 读取 EXIF 时间与云台偏航角
//   - ComputeEpochNs      : 换算成 Unix epoch 纳秒
//   - StampRealmTimestamp : 注入 Xmp.exif.REALM.Timestamp（不改动其它元数据）
//   - ReadRealmTimestamp  : 回读注入结果，用于机上校验
//   - VerifyJpeg          : 轻量结构校验（SOI/段长度/SOS/EOI），不解码像素
namespace h30t_exif {

bool ReadJpegExif(const std::string &path, JpegExifSummary *summary, std::string *error);

// 时间优先级：GPS 日期+时间（UTC）> DateTimeOriginal（按本机时区解释）
bool ComputeEpochNs(const JpegExifSummary &summary, std::int64_t *epoch_ns,
                    bool *from_gps, std::string *error);

// 读取 src，在 SOI 之后插入一个 XMP APP1 段，写出到 dst；src 保持不变
bool StampRealmTimestamp(const std::string &src_path, const std::string &dst_path,
                         std::int64_t epoch_ns, bool has_gimbal_yaw, double gimbal_yaw,
                         std::string *error);

bool ReadRealmTimestamp(const std::string &path, std::int64_t *epoch_ns, std::string *error);

bool VerifyJpeg(const std::string &path, std::string *error);

} // namespace h30t_exif

#endif // H30T_EXIF_WRITER_HPP
