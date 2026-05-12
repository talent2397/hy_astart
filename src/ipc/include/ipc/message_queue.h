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
 * @brief POSIX 消息队列封装 — 用于低频事件通知 (模板类)
 *
 * 基于 Linux mq_open/mq_send/mq_receive 系统调用。
 * 延迟 < 100μs, 适合事件通知 (不适合高频大数据传输, 高频数据请用 SHM)。
 *
 * 使用场景:
 *   - PATH_READY: Planner 通知 Tracker 新路径已就绪
 *   - REPLAN_REQUEST: 请求重规划 (预留)
 *   - EMERGENCY_STOP: 紧急停止 (预留)
 *
 * 使用方式:
 *   // Sender (Writer)
 *   MessageQueue<IPCMessage> mq;
 *   mq.Create("/hy_astar_path_ready", 10);  // 最多 10 条消息
 *   IPCMessage msg;
 *   msg.type = IPCMessageType::PATH_READY;
 *   mq.Send(msg);
 *
 *   // Receiver (Reader)
 *   MessageQueue<IPCMessage> mq;
 *   mq.Open("/hy_astar_path_ready");
 *   IPCMessage msg;
 *   if (mq.TryReceive(msg)) {
 *       // 处理消息
 *   }
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

    /**
     * @brief 创建消息队列 (Writer 端调用)
     * @param name POSIX MQ 名称, 如 "/hy_astar_path_ready"
     * @param max_messages 队列最大消息数 (默认 10)
     *
     * 创建前先 unlink 可能残留的同名队列, 确保干净创建
     */
    bool Create(const std::string& name, int max_messages = 10) {
        name_ = name;
        is_creator_ = true;

        struct mq_attr attr;
        attr.mq_flags = 0;                      // 阻塞模式
        attr.mq_maxmsg = max_messages;           // 最大消息数
        attr.mq_msgsize = sizeof(MsgType);       // 每条消息的固定大小
        attr.mq_curmsgs = 0;

        // 先清理可能残留的同名队列 (上次异常退出可能未 unlink)
        mq_unlink(name_.c_str());

        mqd_ = mq_open(name_.c_str(), O_CREAT | O_RDWR, 0666, &attr);
        if (mqd_ == (mqd_t)-1) {
            perror("mq_open (create)");
            return false;
        }
        return true;
    }

    // ========== 读取者接口 ==========

    /// 打开已存在的消息队列 (Reader 端调用)
    bool Open(const std::string& name, bool read_only = true) {
        name_ = name;
        is_creator_ = false;

        int flags = read_only ? O_RDONLY : O_RDWR;
        mqd_ = mq_open(name_.c_str(), flags);
        if (mqd_ == (mqd_t)-1) {
            return false;  // 尚未创建, 静默返回 false
        }
        return true;
    }

    /// 关闭并清理 (创建者会 unlink)
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

    /**
     * @brief 发送消息 (非阻塞, 队列满时返回 false)
     * @param msg 要发送的消息
     * @param priority 优先级 (0 最低, 数字越大优先级越高)
     */
    bool Send(const MsgType& msg, unsigned int priority = 0) {
        if (mqd_ == (mqd_t)-1) return false;
        int ret = mq_send(mqd_, reinterpret_cast<const char*>(&msg),
                          sizeof(MsgType), priority);
        return ret == 0;
    }

    /**
     * @brief 接收消息 (支持超时)
     * @param timeout_ms -1 = 阻塞等待, 0 = 非阻塞立即返回, >0 = 超时毫秒数
     * @return true 成功收到消息
     */
    bool Receive(MsgType& msg, int timeout_ms = -1) {
        if (mqd_ == (mqd_t)-1) return false;

        struct timespec ts;
        struct timespec* pts = nullptr;

        // 构造超时时间 (绝对时间)
        if (timeout_ms >= 0) {
            clock_gettime(CLOCK_REALTIME, &ts);
            ts.tv_sec += timeout_ms / 1000;
            ts.tv_nsec += (timeout_ms % 1000) * 1000000;
            if (ts.tv_nsec >= 1000000000) {  // 纳秒进位
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

    /// 非阻塞接收 (立即返回, 无消息时返回 false)
    bool TryReceive(MsgType& msg) {
        return Receive(msg, 0);
    }

    bool IsValid() const { return mqd_ != (mqd_t)-1; }

private:
    std::string name_;                  // MQ 名称
    mqd_t mqd_ = (mqd_t)-1;            // mq_open 返回的描述符, -1 表示无效
    bool is_creator_ = false;           // true = 创建者 (负责 unlink)
};

#endif // HYBRID_ASTAR_IPC_MESSAGE_QUEUE_H
