#include "exif_writer.hpp"

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <vector>
#include <unistd.h>

namespace {

const char kXmpNamespace[] = "http://ns.adobe.com/xap/1.0/";
const std::size_t kXmpNamespaceSize = sizeof(kXmpNamespace); // 含结尾 '\0'
const char kExifSignature[] = "Exif\0\0";
const std::size_t kExifSignatureSize = 6;

const uint16_t kTagDateTimeOriginal = 0x9003;
const uint16_t kTagExifIfdPointer = 0x8769;
const uint16_t kTagGpsIfdPointer = 0x8825;
const uint16_t kTagGpsDateStamp = 0x001D;
const uint16_t kTagGpsTimeStamp = 0x0007;

bool ReadU16BE(FILE *fp, uint16_t *value)
{
    uint8_t bytes[2];
    if (std::fread(bytes, 1, 2, fp) != 2) return false;
    *value = static_cast<uint16_t>((bytes[0] << 8) | bytes[1]);
    return true;
}

// 扫描 JPEG 头部，取出第一个 Exif APP1 与第一个 XMP APP1 的负载
bool ScanJpegHeader(FILE *fp,
                    std::vector<uint8_t> *exif_payload,
                    std::vector<uint8_t> *xmp_payload,
                    std::string *error)
{
    uint8_t soi[2];
    if (std::fread(soi, 1, 2, fp) != 2 || soi[0] != 0xFF || soi[1] != 0xD8) {
        *error = "not a JPEG (missing SOI)";
        return false;
    }

    for (;;) {
        int current = std::fgetc(fp);
        if (current == EOF) {
            *error = "unexpected end of file before SOS";
            return false;
        }
        if (current != 0xFF) continue; // 跳过不合规填充字节

        int marker;
        do {
            marker = std::fgetc(fp);
        } while (marker == 0xFF);
        if (marker == EOF) {
            *error = "unexpected end of file while reading marker";
            return false;
        }

        // 无长度的独立标记
        if (marker == 0xD8 || marker == 0x01 || (marker >= 0xD0 && marker <= 0xD7)) continue;
        if (marker == 0xD9 || marker == 0xDA) return true; // 图像数据开始，头部结束

        uint16_t segment_length = 0;
        if (!ReadU16BE(fp, &segment_length) || segment_length < 2) {
            *error = "invalid JPEG segment length";
            return false;
        }
        const std::size_t payload_size = segment_length - 2;

        if (marker == 0xE1 && payload_size > 0) {
            std::vector<uint8_t> payload(payload_size);
            if (std::fread(&payload[0], 1, payload_size, fp) != payload_size) {
                *error = "truncated JPEG APP1 segment";
                return false;
            }
            if (payload_size >= kExifSignatureSize
                && std::memcmp(&payload[0], kExifSignature, kExifSignatureSize) == 0) {
                if (exif_payload && exif_payload->empty()) *exif_payload = payload;
            } else if (payload_size >= kXmpNamespaceSize
                       && std::memcmp(&payload[0], kXmpNamespace, kXmpNamespaceSize) == 0) {
                if (xmp_payload && xmp_payload->empty()) {
                    xmp_payload->assign(payload.begin() + kXmpNamespaceSize, payload.end());
                }
            }
        } else if (payload_size > 0) {
            if (std::fseek(fp, static_cast<long>(payload_size), SEEK_CUR) != 0) {
                *error = "truncated JPEG segment";
                return false;
            }
        }
    }
}

bool OpenAndScan(const std::string &path,
                 std::vector<uint8_t> *exif_payload,
                 std::vector<uint8_t> *xmp_payload,
                 std::string *error)
{
    FILE *fp = std::fopen(path.c_str(), "rb");
    if (fp == NULL) {
        *error = "open failed: " + path;
        return false;
    }
    const bool ok = ScanJpegHeader(fp, exif_payload, xmp_payload, error);
    std::fclose(fp);
    return ok;
}

// 最小 TIFF/IFD 读取器，仅覆盖 EXIF 时间与 GPS 时间所需标签
class TiffReader {
public:
    TiffReader(const uint8_t *data, std::size_t size)
        : data_(data), size_(size), little_endian_(false), valid_(false)
    {
        if (size_ < 8) return;
        if (std::memcmp(data_, "II", 2) == 0) little_endian_ = true;
        else if (std::memcmp(data_, "MM", 2) == 0) little_endian_ = false;
        else return;
        valid_ = true;
    }

    bool valid() const { return valid_; }

    bool U16(std::size_t offset, uint16_t *value) const
    {
        if (offset + 2 > size_) return false;
        const uint8_t *p = data_ + offset;
        *value = little_endian_
            ? static_cast<uint16_t>(p[0] | (p[1] << 8))
            : static_cast<uint16_t>((p[0] << 8) | p[1]);
        return true;
    }

    bool U32(std::size_t offset, uint32_t *value) const
    {
        if (offset + 4 > size_) return false;
        const uint8_t *p = data_ + offset;
        if (little_endian_) {
            *value = static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8)
                | (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
        } else {
            *value = (static_cast<uint32_t>(p[0]) << 24) | (static_cast<uint32_t>(p[1]) << 16)
                | (static_cast<uint32_t>(p[2]) << 8) | static_cast<uint32_t>(p[3]);
        }
        return true;
    }

    bool ReadString(std::size_t offset, std::size_t count, std::string *out) const
    {
        if (offset + count > size_ || count == 0) return false;
        out->assign(reinterpret_cast<const char *>(data_ + offset), count);
        return true;
    }

    bool ReadRational(std::size_t offset, double *out) const
    {
        uint32_t numerator = 0;
        uint32_t denominator = 0;
        if (!U32(offset, &numerator) || !U32(offset + 4, &denominator)) return false;
        if (denominator == 0) return false;
        *out = static_cast<double>(numerator) / static_cast<double>(denominator);
        return true;
    }

private:
    const uint8_t *data_;
    std::size_t size_;
    bool little_endian_;
    bool valid_;
};

struct TiffEntry {
    uint16_t type;
    uint32_t count;
    uint32_t value_offset;    // 数据实际所在偏移（相对 TIFF 头）
    std::size_t value_field;  // 12 字节条目中 value 字段的绝对偏移

    TiffEntry() : type(0), count(0), value_offset(0), value_field(0) {}
};

std::size_t TypeSize(uint16_t type)
{
    switch (type) {
        case 1: case 2: case 6: case 7: return 1; // BYTE/ASCII/SBYTE/UNDEFINED
        case 3: case 8: return 2;                 // SHORT/SSHORT
        case 4: case 9: case 11: return 4;        // LONG/SLONG/FLOAT
        case 5: case 10: case 12: return 8;       // RATIONAL/SRATIONAL/DOUBLE
        default: return 1;
    }
}

bool LookupEntry(const TiffReader &reader, uint32_t ifd_offset, uint16_t tag, TiffEntry *entry)
{
    uint16_t entry_count = 0;
    if (!reader.U16(ifd_offset, &entry_count)) return false;

    for (uint16_t i = 0; i < entry_count; ++i) {
        const std::size_t offset = static_cast<std::size_t>(ifd_offset) + 2 + i * 12;
        uint16_t current_tag = 0;
        if (!reader.U16(offset, &current_tag)) return false;
        if (current_tag != tag) continue;

        entry->type = 0;
        entry->count = 0;
        entry->value_offset = 0;
        reader.U16(offset + 2, &entry->type);
        reader.U32(offset + 4, &entry->count);
        entry->value_field = offset + 8;
        reader.U32(offset + 8, &entry->value_offset);

        const std::size_t total = TypeSize(entry->type) * entry->count;
        if (total <= 4) entry->value_offset = static_cast<uint32_t>(entry->value_field);
        return true;
    }
    return false;
}

bool LookupIfdOffset(const TiffReader &reader, uint32_t ifd_offset, uint16_t pointer_tag,
                     uint32_t *child_offset)
{
    TiffEntry entry;
    if (!LookupEntry(reader, ifd_offset, pointer_tag, &entry)) return false;
    if (entry.type != 4 && entry.type != 3) return false;
    *child_offset = entry.value_offset;
    return true;
}

bool ParseExifAsciiDateTime(const std::string &text,
                            int *year, int *month, int *day,
                            int *hour, int *minute, int *second)
{
    // 形如 "2026:11:17 10:23:45"
    if (text.size() < 19) return false;
    return std::sscanf(text.c_str(), "%4d:%2d:%2d %2d:%2d:%2d",
                       year, month, day, hour, minute, second) == 6;
}

bool ParseExifAsciiDate(const std::string &text, int *year, int *month, int *day)
{
    // 形如 "2026:11:17"
    if (text.size() < 10) return false;
    return std::sscanf(text.c_str(), "%4d:%2d:%2d", year, month, day) == 3;
}

void ParseGimbalYawFromXmp(const std::vector<uint8_t> &xmp, JpegExifSummary *summary)
{
    if (xmp.empty()) return;
    const std::string text(xmp.begin(), xmp.end());
    std::size_t position = text.find("GimbalYawDegree");
    if (position == std::string::npos) return;

    position = text.find('=', position);
    if (position == std::string::npos) return;
    ++position;
    while (position < text.size() && (text[position] == '"' || text[position] == '\'')) ++position;

    errno = 0;
    char *end = NULL;
    const double value = std::strtod(text.c_str() + position, &end);
    if (errno != 0 || end == text.c_str() + position) return;
    const char tail = *end;
    if (tail != '"' && tail != '\'' && tail != '/' && tail != '<' && tail != '>'
        && tail != ' ' && tail != '\t' && tail != '\r' && tail != '\n' && tail != '\0') {
        return;
    }

    summary->has_gimbal_yaw = true;
    summary->gimbal_yaw = value;
}

std::string BuildXmpPacket(std::int64_t epoch_ns, bool has_gimbal_yaw, double gimbal_yaw)
{
    char timestamp[32];
    std::snprintf(timestamp, sizeof(timestamp), "%lld", static_cast<long long>(epoch_ns));

    std::string packet;
    packet += "<?xpacket begin=\"\xEF\xBB\xBF\" id=\"W5M0MpCehiHzreSzNTczkc9d\"?>\n";
    packet += "<x:xmpmeta xmlns:x=\"adobe:ns:meta/\" x:xmptk=\"h30t_downloader\">\n";
    packet += " <rdf:RDF xmlns:rdf=\"http://www.w3.org/1999/02/22-rdf-syntax-ns#\">\n";
    packet += "  <rdf:Description rdf:about=\"\"\n";
    packet += "    xmlns:exif=\"http://ns.adobe.com/exif/1.0/\"\n";
    packet += "    xmlns:drone-dji=\"http://www.dji.com/drone-dji/1.0/\">\n";
    packet += "   <exif:REALM.Timestamp>";
    packet += timestamp;
    packet += "</exif:REALM.Timestamp>\n";
    if (has_gimbal_yaw) {
        char yaw[32];
        std::snprintf(yaw, sizeof(yaw), "%.3f", gimbal_yaw);
        packet += "   <drone-dji:GimbalYawDegree>";
        packet += yaw;
        packet += "</drone-dji:GimbalYawDegree>\n";
    }
    packet += "  </rdf:Description>\n";
    packet += " </rdf:RDF>\n";
    packet += "</x:xmpmeta>\n";
    packet += "<?xpacket end=\"w\"?>";
    return packet;
}

void WriteU16BE(uint8_t *buffer, uint16_t value)
{
    buffer[0] = static_cast<uint8_t>((value >> 8) & 0xFF);
    buffer[1] = static_cast<uint8_t>(value & 0xFF);
}

bool CopyStream(FILE *src, FILE *dst, std::string *error)
{
    std::vector<uint8_t> buffer(1024 * 1024);
    for (;;) {
        const std::size_t read = std::fread(&buffer[0], 1, buffer.size(), src);
        if (read > 0 && std::fwrite(&buffer[0], 1, read, dst) != read) {
            *error = "write failed while copying JPEG body";
            return false;
        }
        if (read < buffer.size()) {
            if (std::ferror(src)) {
                *error = "read failed while copying JPEG body";
                return false;
            }
            return true;
        }
    }
}

} // namespace

JpegExifSummary::JpegExifSummary()
    : has_datetime_original(false), year(0), month(0), day(0), hour(0), minute(0), second(0),
      has_gps_date(false), gps_year(0), gps_month(0), gps_day(0),
      has_gps_time(false), gps_hour(0), gps_minute(0), gps_second(0.0),
      has_gimbal_yaw(false), gimbal_yaw(0.0)
{
}

namespace h30t_exif {

bool ReadJpegExif(const std::string &path, JpegExifSummary *summary, std::string *error)
{
    if (summary == NULL) return false;
    std::string local_error;
    std::string *err = error ? error : &local_error;

    std::vector<uint8_t> exif_payload;
    std::vector<uint8_t> xmp_payload;
    if (!OpenAndScan(path, &exif_payload, &xmp_payload, err)) return false;

    if (!exif_payload.empty() && exif_payload.size() > kExifSignatureSize) {
        const uint8_t *tiff = &exif_payload[kExifSignatureSize];
        const std::size_t tiff_size = exif_payload.size() - kExifSignatureSize;
        TiffReader reader(tiff, tiff_size);
        if (reader.valid()) {
            uint32_t ifd0 = 0;
            if (reader.U32(4, &ifd0)) {
                uint32_t exif_ifd = 0;
                if (LookupIfdOffset(reader, ifd0, kTagExifIfdPointer, &exif_ifd)) {
                    TiffEntry entry;
                    if (LookupEntry(reader, exif_ifd, kTagDateTimeOriginal, &entry)) {
                        std::string text;
                        if (reader.ReadString(entry.value_offset, entry.count, &text)) {
                            summary->has_datetime_original = ParseExifAsciiDateTime(
                                text, &summary->year, &summary->month, &summary->day,
                                &summary->hour, &summary->minute, &summary->second);
                        }
                    }
                }

                uint32_t gps_ifd = 0;
                if (LookupIfdOffset(reader, ifd0, kTagGpsIfdPointer, &gps_ifd)) {
                    TiffEntry entry;
                    if (LookupEntry(reader, gps_ifd, kTagGpsDateStamp, &entry)) {
                        std::string text;
                        if (reader.ReadString(entry.value_offset, entry.count, &text)) {
                            summary->has_gps_date = ParseExifAsciiDate(
                                text, &summary->gps_year, &summary->gps_month, &summary->gps_day);
                        }
                    }
                    if (LookupEntry(reader, gps_ifd, kTagGpsTimeStamp, &entry)
                        && entry.type == 5 && entry.count >= 3) {
                        double hour = 0.0;
                        double minute = 0.0;
                        double second = 0.0;
                        if (reader.ReadRational(entry.value_offset, &hour)
                            && reader.ReadRational(entry.value_offset + 8, &minute)
                            && reader.ReadRational(entry.value_offset + 16, &second)) {
                            summary->has_gps_time = true;
                            summary->gps_hour = static_cast<int>(hour);
                            summary->gps_minute = static_cast<int>(minute);
                            summary->gps_second = second;
                        }
                    }
                }
            }
        }
    }

    ParseGimbalYawFromXmp(xmp_payload, summary);
    return true;
}

bool ComputeEpochNs(const JpegExifSummary &summary, std::int64_t *epoch_ns,
                    bool *from_gps, std::string *error)
{
    if (epoch_ns == NULL) return false;
    if (from_gps) *from_gps = false;

    if (summary.has_gps_date && summary.has_gps_time) {
        struct tm utc;
        std::memset(&utc, 0, sizeof(utc));
        utc.tm_year = summary.gps_year - 1900;
        utc.tm_mon = summary.gps_month - 1;
        utc.tm_mday = summary.gps_day;
        utc.tm_hour = summary.gps_hour;
        utc.tm_min = summary.gps_minute;
        utc.tm_sec = static_cast<int>(summary.gps_second);
        const time_t seconds = timegm(&utc);
        if (seconds > 0) {
            const double fraction = summary.gps_second - static_cast<int>(summary.gps_second);
            *epoch_ns = static_cast<std::int64_t>(seconds) * 1000000000LL
                + static_cast<std::int64_t>(fraction * 1e9);
            if (from_gps) *from_gps = true;
            return true;
        }
    }

    if (summary.has_datetime_original) {
        struct tm local;
        std::memset(&local, 0, sizeof(local));
        local.tm_year = summary.year - 1900;
        local.tm_mon = summary.month - 1;
        local.tm_mday = summary.day;
        local.tm_hour = summary.hour;
        local.tm_min = summary.minute;
        local.tm_sec = summary.second;
        local.tm_isdst = -1;
        const time_t seconds = std::mktime(&local);
        if (seconds > 0) {
            *epoch_ns = static_cast<std::int64_t>(seconds) * 1000000000LL;
            return true;
        }
    }

    if (error) *error = "no usable capture time in EXIF";
    return false;
}

bool StampRealmTimestamp(const std::string &src_path, const std::string &dst_path,
                         std::int64_t epoch_ns, bool has_gimbal_yaw, double gimbal_yaw,
                         std::string *error)
{
    std::string local_error;
    std::string *err = error ? error : &local_error;

    FILE *src = std::fopen(src_path.c_str(), "rb");
    if (src == NULL) {
        *err = "open source failed: " + src_path;
        return false;
    }

    uint8_t soi[2];
    if (std::fread(soi, 1, 2, src) != 2 || soi[0] != 0xFF || soi[1] != 0xD8) {
        std::fclose(src);
        *err = "source is not a JPEG: " + src_path;
        return false;
    }

    const std::string packet = BuildXmpPacket(epoch_ns, has_gimbal_yaw, gimbal_yaw);
    const std::size_t segment_payload = kXmpNamespaceSize + packet.size();
    if (segment_payload + 2 > 0xFFFF) {
        std::fclose(src);
        *err = "XMP packet exceeds JPEG APP1 limit";
        return false;
    }

    FILE *dst = std::fopen(dst_path.c_str(), "wb");
    if (dst == NULL) {
        std::fclose(src);
        *err = "open destination failed: " + dst_path;
        return false;
    }

    bool ok = std::fwrite(soi, 1, 2, dst) == 2;
    if (ok) {
        uint8_t header[4];
        header[0] = 0xFF;
        header[1] = 0xE1;
        WriteU16BE(&header[2], static_cast<uint16_t>(segment_payload + 2));
        ok = std::fwrite(header, 1, sizeof(header), dst) == sizeof(header);
    }
    if (ok) {
        ok = std::fwrite(kXmpNamespace, 1, kXmpNamespaceSize, dst) == kXmpNamespaceSize;
    }
    if (ok) {
        ok = std::fwrite(packet.data(), 1, packet.size(), dst) == packet.size();
    }
    if (ok) {
        // 源文件已读过 SOI，把剩余字节原样搬运
        ok = CopyStream(src, dst, err);
    }

    bool flushed = true;
    if (ok) {
        flushed = std::fflush(dst) == 0 && fsync(fileno(dst)) == 0;
        if (!flushed) *err = "flush stamped JPEG failed";
    }
    std::fclose(dst);
    std::fclose(src);

    if (!ok || !flushed) {
        if (err->empty()) *err = "write stamped JPEG failed";
        std::remove(dst_path.c_str());
        return false;
    }
    return true;
}

bool ReadRealmTimestamp(const std::string &path, std::int64_t *epoch_ns, std::string *error)
{
    if (epoch_ns == NULL) return false;
    std::string local_error;
    std::string *err = error ? error : &local_error;

    std::vector<uint8_t> exif_payload;
    std::vector<uint8_t> xmp_payload;
    if (!OpenAndScan(path, &exif_payload, &xmp_payload, err)) return false;
    if (xmp_payload.empty()) {
        *err = "no XMP segment";
        return false;
    }

    const std::string text(xmp_payload.begin(), xmp_payload.end());
    std::size_t position = text.find("REALM.Timestamp");
    if (position == std::string::npos) {
        *err = "REALM.Timestamp not present";
        return false;
    }
    position = text.find('>', position);
    if (position == std::string::npos) {
        *err = "malformed REALM.Timestamp";
        return false;
    }
    ++position;

    std::int64_t value = 0;
    std::size_t digits = 0;
    while (position < text.size() && text[position] >= '0' && text[position] <= '9') {
        value = value * 10 + (text[position] - '0');
        ++position;
        ++digits;
    }
    if (digits == 0) {
        *err = "REALM.Timestamp is not numeric";
        return false;
    }
    *epoch_ns = value;
    return true;
}

bool VerifyJpeg(const std::string &path, std::string *error)
{
    std::string local_error;
    std::string *err = error ? error : &local_error;

    std::vector<uint8_t> exif_payload;
    std::vector<uint8_t> xmp_payload;
    if (!OpenAndScan(path, &exif_payload, &xmp_payload, err)) return false;

    // 有 SOI 有尾部 EOI 才算结构完整
    FILE *fp = std::fopen(path.c_str(), "rb");
    if (fp == NULL) {
        *err = "open failed: " + path;
        return false;
    }
    if (std::fseek(fp, 0, SEEK_END) != 0) {
        std::fclose(fp);
        *err = "seek failed: " + path;
        return false;
    }
    const long size = std::ftell(fp);
    if (size < 4) {
        std::fclose(fp);
        *err = "file too small";
        return false;
    }
    if (std::fseek(fp, size - 2, SEEK_SET) != 0) {
        std::fclose(fp);
        *err = "seek failed";
        return false;
    }
    uint8_t eoi[2];
    const bool has_eoi = std::fread(eoi, 1, 2, fp) == 2 && eoi[0] == 0xFF && eoi[1] == 0xD9;
    std::fclose(fp);
    if (!has_eoi) {
        *err = "missing EOI marker";
        return false;
    }
    return true;
}

} // namespace h30t_exif
