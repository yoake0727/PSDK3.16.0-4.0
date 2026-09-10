#include "h30t_config.hpp"
#include "app_config.hpp"

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

H30tRtspConfig LoadRtspConfig()
{
    const AppConfig &app=AppConfig::Instance();
    H30tRtspConfig config;
    config.rtsp_url=app.String("h30t.rtsp.url"); config.transport=app.String("h30t.rtsp.transport"); config.max_queue_bytes=app.Size("h30t.rtsp.max_queue_bytes");
    return config;
}

} // namespace h30t_config
