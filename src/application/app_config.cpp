#include "app_config.hpp"
#include <fstream>
#include <stdexcept>

namespace {

// 默认配置文件路径（相对工作目录）。
// 若进程启动目录不是项目根，需要自行调整或保证工作目录正确。
const char *kConfigPath = "src/app_json/app_config.yaml";

} // namespace

/**
 * 构造：读取 YAML 配置文件并解析成扁平 key-value 表。
 *
 * 解析规则（与 localizer.cpp / minio_uploader.cpp 里的极简 YAML 一致）：
 *   1. 逐行读取
 *   2. "#" 之后视为注释，截断
 *   3. 找第一个 ":" 分割 key / value
 *   4. 去掉 key 尾部空白、value 首尾空白（含 \r，兼容 Windows 换行）
 *   5. 非空 key 才写入 values_
 *
 * 特点：
 *   - 只支持扁平结构，不支持嵌套、数组、引号、多行
 *   - 重复 key 后者覆盖前者
 *   - 空 value 也会写入（但 String() 会拒绝空值）
 *
 * 异常：
 *   文件打不开时抛 std::runtime_error
 */
AppConfig::AppConfig()
{
    std::ifstream in(kConfigPath);
    if (!in) {
        throw std::runtime_error(std::string("Cannot open config: ") + kConfigPath);
    }

    std::string line;
    while (std::getline(in, line)) {
        // 去掉 "#" 之后的注释
        std::size_t h = line.find('#');
        if (h != std::string::npos) line.resize(h);

        // 找 "key: value" 的分隔冒号，没有则跳过本行
        std::size_t p = line.find(':');
        if (p == std::string::npos) continue;

        std::string k = line.substr(0, p);
        std::string v = line.substr(p + 1);

        // 去掉 key 尾部空白（含制表符）
        while (!k.empty() && (k.back() == ' ' || k.back() == '\t')) k.pop_back();

        // 去掉 value 首部空白
        while (!v.empty() && (v.front() == ' ' || v.front() == '\t')) v.erase(v.begin());

        // 去掉 value 尾部空白（含 \r，兼容 Windows 换行）
        while (!v.empty() && (v.back() == ' ' || v.back() == '\t' || v.back() == '\r')) v.pop_back();

        // 非空 key 才写入
        if (!k.empty()) values_[k] = v;
    }
}

/**
 * 单例访问：首次调用时构造，之后返回同一实例。
 *
 * 用函数内 static 保证：
 *   - 线程安全初始化（C++11 起）
 *   - 进程退出时自动析构
 *
 * 注意：构造失败会抛异常，无法捕获时会导致进程终止。
 *       若希望容错，需改为返回指针或延迟加载。
 */
const AppConfig &AppConfig::Instance()
{
    static const AppConfig c;
    return c;
}

/**
 * 取字符串值。
 *
 * 异常：
 *   key 不存在或值为空时抛 std::runtime_error
 *
 * 设计意图：空值视为配置错误，强制调用方显式处理，避免静默用空串。
 */
const std::string &AppConfig::String(const std::string &key) const
{
    auto it = values_.find(key);
    if (it == values_.end() || it->second.empty()) {
        throw std::runtime_error("Missing config key: " + key);
    }
    return it->second;
}

/**
 * 取 int 值。
 * 内部调 String()，再 std::stoi。
 * 转换失败（非数字、溢出）会抛 std::invalid_argument / std::out_of_range。
 */
int AppConfig::Int(const std::string &key) const
{
    return std::stoi(String(key));
}

/**
 * 取 size_t 值。
 * 用 stoull 解析后转 size_t，适合端口号、缓冲区大小等非负配置。
 */
std::size_t AppConfig::Size(const std::string &key) const
{
    return static_cast<std::size_t>(std::stoull(String(key)));
}

/**
 * 取 double 值。
 * 内部调 String()，再 std::stod。
 * 适合焦距、阈值、比例等浮点配置。
 */
double AppConfig::Double(const std::string &key) const
{
    return std::stod(String(key));
}