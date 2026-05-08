#ifndef HYBRID_ASTAR_IPC_DATA_BUS_H
#define HYBRID_ASTAR_IPC_DATA_BUS_H

#include "shared_memory.h"
#include "message_queue.h"

#include <string>
#include <functional>
#include <thread>
#include <atomic>
#include <cstring>

/**
 * @brief DataBusWriter — 写入共享内存
 *
 * 使用 mmap 直接映射，零拷贝写入。
 */
template<typename T>
class DataBusWriter {
public:
    DataBusWriter() = default;

    bool Open(const std::string& shm_name) {
        shm_name_ = shm_name;
        return shm_.Create(shm_name_);
    }

    // 写入数据（memcpy 到共享内存）
    bool Write(const T& data) {
        if (!shm_.IsValid()) return false;
        T* target = shm_.Get();
        // 保存旧 sequence，避免被 memcpy 覆盖为 0
        uint64_t old_seq = target->sequence;
        target->is_valid = 0;               // 写入中标记为无效
        std::memcpy(target, &data, sizeof(T));
        target->sequence = old_seq + 1;     // 递增 sequence
        target->is_valid = 1;               // 写入完成
        return true;
    }

    const std::string& Name() const { return shm_name_; }

private:
    std::string shm_name_;
    SharedMemory<T> shm_;
};

/**
 * @brief DataBusReader — 读取共享内存
 *
 * 通过 sequence 号检测是否有新数据。
 */
template<typename T>
class DataBusReader {
public:
    DataBusReader() = default;

    bool Open(const std::string& shm_name) {
        shm_name_ = shm_name;
        return shm_.Open(shm_name_, false);
    }

    // 以指定映射大小打开（用于可变长度数据，如地图 header+grid）
    bool Open(const std::string& shm_name, size_t map_size) {
        shm_name_ = shm_name;
        return shm_.Open(shm_name_, map_size, false);
    }

    // 读取最新数据（检测 sequence 变化）
    bool ReadLatest(T& data) {
        if (!shm_.IsValid()) return false;
        const T* ptr = shm_.Get();
        if (!ptr->is_valid) return false;
        if (ptr->sequence == last_sequence_) return false;

        std::memcpy(&data, ptr, sizeof(T));
        last_sequence_ = ptr->sequence;
        return true;
    }

    // 阻塞等待新数据
    bool WaitLatest(T& data, int timeout_ms = 100) {
        auto start = std::chrono::steady_clock::now();
        while (true) {
            if (ReadLatest(data)) return true;
            auto elapsed = std::chrono::steady_clock::now() - start;
            if (std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count() >= timeout_ms)
                return false;
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    }

    bool HasNewData() const {
        if (!shm_.IsValid()) return false;
        const T* ptr = shm_.Get();
        return ptr->is_valid && ptr->sequence != last_sequence_;
    }

    bool IsValid() const { return shm_.IsValid(); }
    const std::string& Name() const { return shm_name_; }

    void Close() { shm_.Close(); last_sequence_ = 0; }

    // 获取底层 SHM 的原始指针（用于可变长度数据，如地图 grid）
    const uint8_t* GetRaw() const { return shm_.GetRaw(); }
    const T* Get() const { return shm_.Get(); }

private:
    std::string shm_name_;
    SharedMemory<T> shm_;
    uint64_t last_sequence_ = 0;
};

#endif // HYBRID_ASTAR_IPC_DATA_BUS_H
