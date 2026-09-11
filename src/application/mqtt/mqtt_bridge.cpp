#include "mqtt_bridge.hpp"
#include <dji_logger.h>
#include <cstring>

MqttBridge::MqttBridge(std::chrono::steady_clock::time_point app_start_time)
    : app_start_time_(app_start_time)
{
}

MqttBridge::~MqttBridge()
{
    close();
}

bool MqttBridge::init(const MqttConfig &cfg)
{
    cfg_ = cfg;
    int rc = mosquitto_lib_init();
    if (rc != MOSQ_ERR_SUCCESS)
    {
        USER_LOG_ERROR("[MQTT] mosquitto_lib_init failed: %s", mosquitto_strerror(rc));
        return false;
    }

    // 将 this 作为 userdata 传入 mosquitto_new
    mosq_ = mosquitto_new(cfg.client_id.c_str(), true, this);
    if (!mosq_)
    {
        USER_LOG_ERROR("[MQTT] mosquitto_new failed (maybe OOM or invalid client_id)");
        return false;
    }

    if (!cfg.username.empty())
    {
        int urc = mosquitto_username_pw_set(
            mosq_, cfg.username.c_str(),
            cfg.password.empty() ? nullptr : cfg.password.c_str());
        if (urc != MOSQ_ERR_SUCCESS)
        {
            USER_LOG_ERROR("[MQTT] set username/password failed: %s", mosquitto_strerror(urc));
        }
    }

    // 注册回调函数
    mosquitto_connect_callback_set(mosq_, on_connect);
    mosquitto_message_callback_set(mosq_, on_message);
    mosquitto_subscribe_callback_set(mosq_, on_subscribe);

    int connect_rc = mosquitto_connect(mosq_, cfg.host.c_str(), cfg.port, cfg.keepalive);
    if (connect_rc != MOSQ_ERR_SUCCESS)
    {
        USER_LOG_ERROR("[MQTT] connect failed to %s:%d - %s", cfg.host.c_str(), cfg.port, mosquitto_strerror(connect_rc));
        mosquitto_destroy(mosq_);
        mosq_ = nullptr;
        return false;
    }

    int loop_rc = mosquitto_loop_start(mosq_);
    if (loop_rc != MOSQ_ERR_SUCCESS)
    {
        USER_LOG_ERROR("[MQTT] loop start failed: %s", mosquitto_strerror(loop_rc));
        mosquitto_destroy(mosq_);
        mosq_ = nullptr;
        return false;
    }

    USER_LOG_INFO("[MQTT] connected to %s:%d as %s", cfg.host.c_str(), cfg.port, cfg.client_id.c_str());
    return true;
}

bool MqttBridge::publish(const std::string &topic, const std::string &payload, int qos, bool retain)
{
    if (!mosq_)
        return false;
    int rc = mosquitto_publish(mosq_, nullptr, topic.c_str(), static_cast<int>(payload.size()), payload.c_str(), qos, retain);

    if (rc != MOSQ_ERR_SUCCESS)
    {
        USER_LOG_ERROR("[MQTT] publish failed: %s", mosquitto_strerror(rc));
    }
    return rc == MOSQ_ERR_SUCCESS;
}

void MqttBridge::close()
{
    if (mosq_)
    {
        mosquitto_loop_stop(mosq_, true);
        mosquitto_disconnect(mosq_);
        mosquitto_destroy(mosq_);
        mosq_ = nullptr;
    }
    mosquitto_lib_cleanup();
}

bool MqttBridge::subscribe(const std::string &topic, int qos)
{
    if (!mosq_)
        return false;
    // 主题存入列表并去重，避免重复订阅
    for (const auto& t : sub_topics_) {
        if (t == topic) return true;
    }
    sub_topics_.push_back(topic);

    int rc = mosquitto_subscribe(mosq_, nullptr, topic.c_str(), qos);
    if (rc != MOSQ_ERR_SUCCESS)
    {
        USER_LOG_ERROR("[MQTT] subscribe failed: %s", mosquitto_strerror(rc));
        return false;
    }
    return true;
}

void MqttBridge::set_message_callback(MessageCallback cb)
{
    message_callback_ = cb;
}

bool MqttBridge::clear_retained_message(const std::string& topic) {
    // 发布空消息并设置保留标志以清除保留消息
    return publish(topic, "", 1, true);
}

// 静态回调函数
void MqttBridge::on_connect(struct mosquitto *mosq, void *obj, int rc)
{
    MqttBridge *self = static_cast<MqttBridge *>(obj);
    if (!self)
        return;

    if (rc == MOSQ_ERR_SUCCESS)
    {
        USER_LOG_INFO("[MQTT] Connected to MQTT broker");
        // 重连后自动恢复所有订阅
        self->subscribe_all_topics();
        // 连接成功后清除保留消息
        std::string ctl_topic = "drone/" + self->cfg_.drone_id + "/psdk/command/ctl";
        self->clear_retained_message(ctl_topic);
    }
    else
    {
        USER_LOG_ERROR("[MQTT] Connection failed: %s", mosquitto_strerror(rc));
    }
}

void MqttBridge::on_message(struct mosquitto *mosq, void *obj, const struct mosquitto_message *message)
{
    MqttBridge *self = static_cast<MqttBridge *>(obj);
    if (!self || !message)
        return;

    std::string topic;
    if (message->topic)
        topic = std::string(message->topic);

    std::string payload;
    if (message->payload && message->payloadlen > 0)
    {
        payload.assign(static_cast<const char *>(message->payload), message->payloadlen);
    }
    else
    {
        payload.clear();
    }

    // 计算应用程序运行时间
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - self->app_start_time_).count();

    // 如果用户注册了回调，则调用
    if (self->message_callback_)
    {
        try
        {
            // 传递三个参数：topic, payload 和 retained 标志
            self->message_callback_(topic, payload, (message->retain == 1));
        }
        catch (const std::exception &e)
        {
            USER_LOG_ERROR("[MQTT] message callback threw: %s", e.what());
        }
        catch (...)
        {
            USER_LOG_ERROR("[MQTT] message callback threw unknown exception");
        }
    }
}

void MqttBridge::on_subscribe(struct mosquitto *mosq, void *obj, int mid, int qos_count, const int *granted_qos)
{
    MqttBridge *self = static_cast<MqttBridge *>(obj);
    if (!self)
        return;

    USER_LOG_INFO("[MQTT] Subscribed to topic with QoS: %d", granted_qos ? granted_qos[0] : 0);
}

// ========== 新增：自动订阅所有保存的主题 ==========
void MqttBridge::subscribe_all_topics() {
    if (!mosq_ || sub_topics_.empty()) {
        return;
    }

    // 遍历所有主题，重新订阅
    for (const std::string& topic : sub_topics_) {
        int rc = mosquitto_subscribe(mosq_, nullptr, topic.c_str(), 0);
        if (rc == MOSQ_ERR_SUCCESS) {
            USER_LOG_INFO("[MQTT] 恢复订阅主题: %s", topic.c_str());
        } else {
            USER_LOG_ERROR("[MQTT] 订阅失败: %s 错误: %s", topic.c_str(), mosquitto_strerror(rc));
        }
    }
}

