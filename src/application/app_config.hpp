#pragma once
#include <cstddef>
#include <map>
#include <string>
class AppConfig {
public:
    static const AppConfig &Instance();
    const std::string &String(const std::string &key) const;
    int Int(const std::string &key) const;
    std::size_t Size(const std::string &key) const;
    double Double(const std::string &key) const;
private:
    AppConfig();
    std::map<std::string, std::string> values_;
};
