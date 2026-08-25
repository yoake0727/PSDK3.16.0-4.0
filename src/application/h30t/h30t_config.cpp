#include "h30t_config.hpp"

#include <cerrno>
#include <climits>
#include <cstdlib>

namespace {

std::string EnvironmentOrDefault(const char *name, const char *fallback)
{
    const char *value = std::getenv(name);
    return value != NULL && value[0] != '\0' ? value : fallback;
}

std::size_t ParseQueueBytes(const char *value, std::size_t fallback)
{
    if (value == NULL || value[0] == '\0') return fallback;
    errno = 0;
    char *end = NULL;
    const unsigned long long parsed = std::strtoull(value, &end, 10);
    if (errno != 0 || end == value || *end != '\0' || parsed == 0
        || parsed > static_cast<unsigned long long>(SIZE_MAX)) {
        return fallback;
    }
    return static_cast<std::size_t>(parsed);
}

} // namespace

namespace h30t_config {

std::string RedactRtspUrl(const std::string &url)
{
    const std::size_t scheme_end = url.find("://");
    if (scheme_end == std::string::npos) return url;
    const std::size_t authority_start = scheme_end + 3;
    const std::size_t at = url.find('@', authority_start);
    const std::size_t colon = url.find(':', authority_start);
    if (at == std::string::npos || colon == std::string::npos || colon >= at) return url;
    return url.substr(0, colon + 1) + "***" + url.substr(at);
}

bool IsValidRtspUrl(const std::string &url)
{
    const std::string prefix("rtsp://");
    if (url.compare(0, prefix.size(), prefix) != 0 || url.size() <= prefix.size()) return false;
    const std::size_t authority_end = url.find('/', prefix.size());
    const std::size_t length =
        (authority_end == std::string::npos ? url.size() : authority_end) - prefix.size();
    return length > 0;
}

bool IsValidRtspTransport(const std::string &transport)
{
    return transport == "tcp" || transport == "udp";
}

bool ValidateRtspConfig(const H30tRtspConfig &config)
{
    return IsValidRtspUrl(config.rtsp_url)
        && IsValidRtspTransport(config.transport)
        && config.max_queue_bytes > 0;
}

bool ParseEnabledFlag(const char *value)
{
    return value != NULL && std::string(value) == "1";
}

int ParseMount(const char *value)
{
    if (value == NULL || value[0] == '\0') return 1;
    const std::string text(value);
    return text == "1" ? 1 : text == "2" ? 2 : text == "3" ? 3 : 0;
}

H30tRtspConfig LoadRtspConfigFromEnvironment()
{
    H30tRtspConfig config;
    config.rtsp_url = EnvironmentOrDefault(
        "H30T_RTSP_URL", "rtsp://127.0.0.1:8554/h30t");
    config.transport = EnvironmentOrDefault("H30T_RTSP_TRANSPORT", "tcp");
    config.max_queue_bytes = ParseQueueBytes(
        std::getenv("H30T_RTSP_MAX_QUEUE_BYTES"), config.max_queue_bytes);
    return config;
}

} // namespace h30t_config
