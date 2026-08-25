#pragma once

#include <unordered_map>
#include <string>
#include <mutex>

class SimpleCache {
public:
    SimpleCache() = default;

    // 添加字符串版本的 put 和 get 方法
    void put(const std::string &key, const std::string &value) {
        std::lock_guard<std::mutex> lk(mtx_);
        str_cache_[key] = value;
    }

    bool get(const std::string &key, std::string &outValue) const {
        std::lock_guard<std::mutex> lk(mtx_);
        auto it = str_cache_.find(key);
        if (it != str_cache_.end()) {
            outValue = it->second;
            return true;
        }
        return false;
    }

    std::string get_or_default(const std::string &key, const std::string &def = "") const {
        std::lock_guard<std::mutex> lk(mtx_);
        auto it = str_cache_.find(key);
        if (it != str_cache_.end()) return it->second;
        return def;
    }

    // 保留原有的整数版本方法（如果需要）
    void put(const std::string &key, int value) {
        std::lock_guard<std::mutex> lk(mtx_);
        cache_[key] = value;
    }

    bool get(const std::string &key, int &outValue) const {
        std::lock_guard<std::mutex> lk(mtx_);
        auto it = cache_.find(key);
        if (it != cache_.end()) {
            outValue = it->second;
            return true;
        }
        return false;
    }

    int get_or_default(const std::string &key, int def = 0) const {
        std::lock_guard<std::mutex> lk(mtx_);
        auto it = cache_.find(key);
        if (it != cache_.end()) return it->second;
        return def;
    }

    // 删除 key
    void remove(const std::string &key) {
        std::lock_guard<std::mutex> lk(mtx_);
        cache_.erase(key);
        str_cache_.erase(key);
    }

    // 判断是否存在
    bool contains(const std::string &key) {
        std::lock_guard<std::mutex> lk(mtx_);
        return cache_.find(key) != cache_.end() || str_cache_.find(key) != str_cache_.end();
    }

private:
    mutable std::mutex mtx_;
    std::unordered_map<std::string, int> cache_;
    std::unordered_map<std::string, std::string> str_cache_; // 新增字符串缓存
};