#ifndef HYBRID_ASTAR_IPC_MESSAGE_QUEUE_H
#define HYBRID_ASTAR_IPC_MESSAGE_QUEUE_H

#include <string>
#include <cstring>
#include <mqueue.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <stdexcept>
#include <cerrno>

/**
 * @brief POSIX 消息队列封装（用于事件通知）
 *
 * 轻量级，延迟 < 100μs。
 * 用于：PATH_READY, REPLAN_REQUEST, EMERGENCY_STOP 等事件。
 */
template<typename MsgType>
class MessageQueue {
public:
    MessageQueue() = default;

    ~MessageQueue() {
        Close();
    }

    MessageQueue(const MessageQueue&) = delete;
    MessageQueue& operator=(const MessageQueue&) = delete;

    // ========== 创建者接口 ==========
    // 创建消息队列（writer 端调用）
    bool Create(const std::string& name, int max_messages = 10) {
        name_ = name;
        is_creator_ = true;

        struct mq_attr attr;
        attr.mq_flags = 0;
        attr.mq_maxmsg = max_messages;
        attr.mq_msgsize = sizeof(MsgType);
        attr.mq_curmsgs = 0;

        // 先尝试 unlink 可能残留的
        mq_unlink(name_.c_str());

        mqd_ = mq_open(name_.c_str(), O_CREAT | O_RDWR, 0666, &attr);
        if (mqd_ == (mqd_t)-1) {
            perror("mq_open (create)");
            return false;
        }
        return true;
    }

    // ========== 读取者接口 ==========
    // 打开已存在的消息队列（reader 端调用）
    bool Open(const std::string& name, bool read_only = true) {
        name_ = name;
        is_creator_ = false;

        int flags = read_only ? O_RDONLY : O_RDWR;
        mqd_ = mq_open(name_.c_str(), flags);
        if (mqd_ == (mqd_t)-1) {
            return false;
        }
        return true;
    }

    // 关闭
    void Close() {
        if (mqd_ != (mqd_t)-1) {
            mq_close(mqd_);
            mqd_ = (mqd_t)-1;
        }
        if (is_creator_ && !name_.empty()) {
            mq_unlink(name_.c_str());
        }
    }

    // ========== 消息操作 ==========

    // 发送消息（非阻塞，队列满时返回 false）
    bool Send(const MsgType& msg, unsigned int priority = 0) {
        if (mqd_ == (mqd_t)-1) return false;
        int ret = mq_send(mqd_, reinterpret_cast<const char*>(&msg),
                          sizeof(MsgType), priority);
        return ret == 0;
    }

    // 接收消息（timeout_ms: -1=阻塞, 0=非阻塞, >0=超时ms）
    bool Receive(MsgType& msg, int timeout_ms = -1) {
        if (mqd_ == (mqd_t)-1) return false;

        struct timespec ts;
        struct timespec* pts = nullptr;

        if (timeout_ms >= 0) {
            clock_gettime(CLOCK_REALTIME, &ts);
            ts.tv_sec += timeout_ms / 1000;
            ts.tv_nsec += (timeout_ms % 1000) * 1000000;
            if (ts.tv_nsec >= 1000000000) {
                ts.tv_sec += 1;
                ts.tv_nsec -= 1000000000;
            }
            pts = &ts;
        }

        ssize_t ret = mq_timedreceive(mqd_, reinterpret_cast<char*>(&msg),
                                      sizeof(MsgType), nullptr, pts);
        if (ret < 0) {
            if (errno == ETIMEDOUT) return false;
            return false;
        }
        return true;
    }

    // 非阻塞接收
    bool TryReceive(MsgType& msg) {
        return Receive(msg, 0);
    }

    bool IsValid() const { return mqd_ != (mqd_t)-1; }

private:
    std::string name_;
    mqd_t mqd_ = (mqd_t)-1;
    bool is_creator_ = false;
};

#endif // HYBRID_ASTAR_IPC_MESSAGE_QUEUE_H
