#ifndef H30T_DOWNLOAD_CONFIG_HPP
#define H30T_DOWNLOAD_CONFIG_HPP

#include "download_types.hpp"

#include <string>

namespace h30t_download_config {

// 从环境变量加载配置（未设置的项沿用默认值）
H30tDownloadConfig LoadFromEnvironment();

// 校验配置；失败时 error 给出原因
bool Validate(const H30tDownloadConfig &config, std::string *error);

// 路径拼接，避免出现重复的 '/'
std::string JoinPath(const std::string &base, const std::string &name);

} // namespace h30t_download_config

#endif // H30T_DOWNLOAD_CONFIG_HPP
