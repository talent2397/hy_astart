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
 * @brief DataBusWriter — 共享内存写入器 (模板类)
 *
 * 封装 SHM 的创建和写入逻辑。
 * 使用 mmap 直接映射物理内存, 写入零拷贝 (一次 memcpy)。
 *
 * 模板参数 T: SHM 数据结构类型 (如 VehicleStateData, PlannerPathData 等)
 *
 * 使用示例:
 *   DataBusWriter<VehicleStateData> writer;
 *   writer.Open(SHM_VEHICLE_STATE);
 *   VehicleStateData data;
 *   // ... 填充 data ...
 *   writer.Write(data);
 */
template<typename T>
class DataBusWriter {
public:
    DataBusWriter() = default;

    /// 创建并打开共享内存段 (writer 端必须是创建者)
    bool Open(const std::string& shm_name) {
        shm_name_ = shm_name;
        return shm_.Create(shm_name_);
    }

    /**
     * @brief 将数据写入共享内存
     *
     * 同步协议:
     *   1. 保存当前 sequence (避免被 memcpy 覆盖为 0)
     *   2. 设 is_valid = 0 (标记写入中, 读取端应跳过)
     *   3. memcpy 整个结构体到 SHM
     *   4. sequence = old_seq + 1 (递增, 读取端据此检测新数据)
     *   5. is_valid = 1 (标记写入完成)
     */
    bool Write(const T& data) {
        if (!shm_.IsValid()) return false;
        T* target = shm_.Get();
        // 保存旧 sequence, 避免被 memcpy 覆盖为 0
        // Bug #4 修复: 之前 memcpy(data→SHM) 把 source 的 sequence=0 复制进去, 导致永远读到 sequence=1
        uint64_t old_seq = target->sequence;
        target->is_valid = 0;               // 写入中标记为无效
        std::memcpy(target, &data, sizeof(T));
        target->sequence = old_seq + 1;     // 递增 sequence, 通知读取端有新数据
        target->is_valid = 1;               // 写入完成
        return true;
    }

    const std::string& Name() const { return shm_name_; }

private:
    std::string shm_name_;
    SharedMemory<T> shm_;
};

/**
 * @brief DataBusReader — 共享内存读取器 (模板类)
 *
 * 通过 sequence 号变化检测是否有新数据, 无锁读取。
 *
 * 核心逻辑:
 *   1. 检查 is_valid (写入方是否正在写)
 *   2. 检查 sequence 是否变化 (上次读到序列号 != 当前序列号)
 *   3. 如果变化, memcpy 到本地 buffer, 更新 last_sequence_
 *
 * 使用示例:
 *   DataBusReader<VehicleStateData> reader;
 *   reader.Open(SHM_VEHICLE_STATE);
 *   VehicleStateData data;
 *   if (reader.ReadLatest(data)) {
 *       // 有新数据, 使用 data
 *   }
 */
template<typename T>
class DataBusReader {
public:
    DataBusReader() = default;

    /// 打开共享内存段 (reader 端, 等待 writer 创建)
    bool Open(const std::string& shm_name) {
        shm_name_ = shm_name;
        return shm_.Open(shm_name_, false);
    }

    /// 以指定映射大小打开 (用于可变长度数据, 如地图 header+grid)
    bool Open(const std::string& shm_name, size_t map_size) {
        shm_name_ = shm_name;
        return shm_.Open(shm_name_, map_size, false);
    }

    /**
     * @brief 非阻塞读取最新数据
     * @return true 表示有新数据 (sequence 已变化)
     *
     * 典型用法: 高频循环中调用, 有新数据时处理, 无新数据时跳过
     */
    bool ReadLatest(T& data) {
        if (!shm_.IsValid()) return false;
        const T* ptr = shm_.Get();
        if (!ptr->is_valid) return false;           // 写入方正在写, 跳过
        if (ptr->sequence == last_sequence_) return false;  // 无新数据

        std::memcpy(&data, ptr, sizeof(T));
        last_sequence_ = ptr->sequence;
        return true;
    }

    /**
     * @brief 阻塞等待新数据 (带超时)
     * @param timeout_ms 超时毫秒数, 0 表示立即返回
     * @return true 表示在超时前收到新数据
     */
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

    /// 检查是否有新数据 (不读取, 仅检测)
    bool HasNewData() const {
        if (!shm_.IsValid()) return false;
        const T* ptr = shm_.Get();
        return ptr->is_valid && ptr->sequence != last_sequence_;
    }

    bool IsValid() const { return shm_.IsValid(); }
    const std::string& Name() const { return shm_name_; }

    void Close() { shm_.Close(); last_sequence_ = 0; }

    /// 获取底层 SHM 的原始字节指针 (用于可变长度数据, 如读取地图 grid)
    const uint8_t* GetRaw() const { return shm_.GetRaw(); }
    const T* Get() const { return shm_.Get(); }

private:
    std::string shm_name_;
    SharedMemory<T> shm_;
    uint64_t last_sequence_ = 0;  // 上次读到的 sequence, 用于检测新数据
};

#endif // HYBRID_ASTAR_IPC_DATA_BUS_H
