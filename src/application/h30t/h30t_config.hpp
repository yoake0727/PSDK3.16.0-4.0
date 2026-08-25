#ifndef H30T_CONFIG_HPP
#define H30T_CONFIG_HPP

#include "h30t_types.hpp"

#include <string>

namespace h30t_config {

std::string RedactRtspUrl(const std::string &url);
bool IsValidRtspUrl(const std::string &url);
bool IsValidRtspTransport(const std::string &transport);
bool ValidateRtspConfig(const H30tRtspConfig &config);
bool ParseEnabledFlag(const char *value);
int ParseMount(const char *value);
H30tRtspConfig LoadRtspConfigFromEnvironment();

} // namespace h30t_config

#endif // H30T_CONFIG_HPP
