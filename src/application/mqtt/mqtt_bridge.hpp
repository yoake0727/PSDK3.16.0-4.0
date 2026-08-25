#pragma once

#include <mosquitto.h>
#include <string>
#include <cstdint>
#include <functional>
#include <chrono>
#include <vector>
#include <mutex>

struct MqttConfig
{
    std::string host = "192.168.144.125";
    int port = 1883;
    int keepalive = 60;
    std::string username = "mqttuser";
    std::string password = "mqtt";
    std::string client_id = "M400_MANIFOLD3";
    std::string drone_id = "1581FBDBW256500A2NXJ";
};

class MqttBridge
{
public:
    // 构造函数接受应用启动时间（用于忽略初期消息）
    explicit MqttBridge(std::chrono::steady_clock::time_point app_start_time);
    ~MqttBridge();

    // 初始化并连接
    bool init(const MqttConfig &cfg);

    // 发布消息，返回 true 表示成功（仅表示发送成功，不代表 broker 已收到）
    bool publish(const std::string &topic, const std::string &payload,
                 int qos = 1, bool retain = false);

    // 关闭连接并清理
    void close();

    // 获取当前配置（只读）
    const MqttConfig &cfg() const { return cfg_; }

    // 订阅主题（线程安全）
    bool subscribe(const std::string &topic, int qos = 1);

    // 重连后自动恢复所有订阅（内部使用）
    void subscribe_all_topics();

    // 设置收到消息时的回调（在 mosquitto 线程中执行，注意线程安全）
    using MessageCallback = std::function<void(const std::string &topic,
                                                const std::string &payload,
                                                bool retained)>;
    void set_message_callback(MessageCallback cb);

    // 清除指定主题的保留消息（发布空 payload 且 retain=true）
    bool clear_retained_message(const std::string &topic);

private:
    // mosquitto 静态回调函数
    static void on_connect(struct mosquitto *mosq, void *obj, int rc);
    static void on_message(struct mosquitto *mosq, void *obj,
                           const struct mosquitto_message *message);
    static void on_subscribe(struct mosquitto *mosq, void *obj,
                             int mid, int qos_count, const int *granted_qos);

    MqttConfig cfg_;
    mosquitto *mosq_ = nullptr;
    MessageCallback message_callback_;
    std::chrono::steady_clock::time_point app_start_time_;

    // 保存已订阅的主题列表，用于重连后自动恢复
    std::vector<std::string> sub_topics_;
    std::mutex sub_topics_mutex_;
};