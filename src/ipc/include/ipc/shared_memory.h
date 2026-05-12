#ifndef HYBRID_ASTAR_IPC_SHARED_MEMORY_H
#define HYBRID_ASTAR_IPC_SHARED_MEMORY_H

#include <string>
#include <stdexcept>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <new>

/**
 * @brief POSIX 共享内存底层封装 (模板类)
 *
 * 基于 shm_open + mmap, 提供零拷贝的进程间数据共享。
 *
 * 设计要点:
 *   - Writer 端调用 Create() 创建 SHM 段并设置大小
 *   - Reader 端调用 Open() 映射已存在的 SHM 段
 *   - 通过 mmap 直接映射到进程虚拟地址空间, 读写与普通内存一样快
 *   - 析构时自动 unlink (仅创建者)
 *
 * 使用方式:
 *   // Writer
 *   SharedMemory<MyData> shm;
 *   shm.Create("/my_shm");       // 创建 (O_CREAT | O_RDWR)
 *   shm->field = value;          // 直接通过指针访问
 *
 *   // Reader
 *   SharedMemory<MyData> shm;
 *   shm.Open("/my_shm", false);  // 只读打开
 *   auto val = shm->field;       // 直接读取
 */
template<typename T>
class SharedMemory {
public:
    SharedMemory() = default;

    ~SharedMemory() {
        Close();
    }

    SharedMemory(const SharedMemory&) = delete;
    SharedMemory& operator=(const SharedMemory&) = delete;

    // ========== 创建者接口 ==========

    /**
     * @brief 创建共享内存段 (Writer 端调用)
     * @param name POSIX SHM 名称, 如 "/hy_astar_vehicle_state"
     * @param mode 权限位 (默认 0666 = 所有用户可读写)
     * @return true 成功
     *
     * 流程: shm_open(O_CREAT) → ftruncate(设为 sizeof(T)) → mmap → placement new 初始化为零
     */
    bool Create(const std::string& name, mode_t mode = 0666) {
        name_ = name;
        is_creator_ = true;

        shm_fd_ = shm_open(name_.c_str(), O_CREAT | O_RDWR, mode);
        if (shm_fd_ < 0) {
            perror("shm_open (create)");
            return false;
        }

        // 设置 SHM 段大小为 sizeof(T)
        if (ftruncate(shm_fd_, sizeof(T)) < 0) {
            perror("ftruncate");
            return false;
        }

        // mmap 映射到进程地址空间, MAP_SHARED 表示修改对其他进程可见
        ptr_ = static_cast<T*>(mmap(nullptr, sizeof(T),
                                    PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_, 0));
        if (ptr_ == MAP_FAILED) {
            perror("mmap");
            return false;
        }

        // 用 placement new 将内存初始化为零 (value-initialization)
        new (ptr_) T{};
        return true;
    }

    // ========== 读取者接口 ==========

    /// 打开已存在的 SHM 段 (默认映射 sizeof(T) 字节, 只读)
    bool Open(const std::string& name, bool read_only = true) {
        return Open(name, sizeof(T), read_only);
    }

    /**
     * @brief 打开已存在的 SHM 段, 指定映射大小
     * @param map_size 映射的字节数 (用于可变长度数据如地图)
     *
     * 注意: 打开失败时静默返回 false (不打印错误),
     * 因为 Reader 可能在 Writer 之前启动, 需要轮询重试
     */
    bool Open(const std::string& name, size_t map_size, bool read_only = true) {
        name_ = name;
        is_creator_ = false;

        int flags = read_only ? O_RDONLY : O_RDWR;
        shm_fd_ = shm_open(name_.c_str(), flags, 0);
        if (shm_fd_ < 0) {
            // 尚未创建, 静默返回 false (调用方会轮询重试)
            return false;
        }

        int prot = read_only ? PROT_READ : (PROT_READ | PROT_WRITE);
        ptr_ = static_cast<T*>(mmap(nullptr, map_size, prot,
                                    MAP_SHARED, shm_fd_, 0));
        if (ptr_ == MAP_FAILED) {
            perror("mmap");
            return false;
        }

        mapped_size_ = map_size;
        return true;
    }

    /// 关闭并清理资源 (创建者会 unlink)
    void Close() {
        if (ptr_ && ptr_ != MAP_FAILED) {
            size_t unmap_size = (mapped_size_ > 0) ? mapped_size_ : sizeof(T);
            munmap(ptr_, unmap_size);
            ptr_ = nullptr;
        }
        if (shm_fd_ >= 0) {
            close(shm_fd_);
            shm_fd_ = -1;
        }
        // 仅创建者负责 unlink (删除 SHM 段)
        if (is_creator_ && !name_.empty()) {
            shm_unlink(name_.c_str());
        }
    }

    /// 手动 unlink SHM 段 (用于 cleanup)
    static void Unlink(const std::string& name) {
        shm_unlink(name.c_str());
    }

    // ========== 指针访问 ==========
    T* operator->() { return ptr_; }
    const T* operator->() const { return ptr_; }
    T& operator*() { return *ptr_; }
    const T& operator*() const { return *ptr_; }
    T* Get() { return ptr_; }
    const T* Get() const { return ptr_; }

    /// 获取映射区域首地址 (用于可变长度数据, 如读取地图 header 后的 grid[])
    uint8_t* GetRaw() { return reinterpret_cast<uint8_t*>(ptr_); }
    const uint8_t* GetRaw() const { return reinterpret_cast<const uint8_t*>(ptr_); }

    size_t GetMappedSize() const { return mapped_size_ > 0 ? mapped_size_ : sizeof(T); }

    bool IsValid() const { return ptr_ != nullptr && ptr_ != MAP_FAILED; }
    const std::string& Name() const { return name_; }

private:
    std::string name_;              // SHM 名称 (如 "/hy_astar_vehicle_state")
    int shm_fd_ = -1;               // shm_open 返回的文件描述符
    T* ptr_ = nullptr;              // mmap 映射的虚拟地址
    size_t mapped_size_ = 0;        // 实际映射的字节数 (0 表示默认 = sizeof(T))
    bool is_creator_ = false;       // true = 创建者 (负责 ftruncate + unlink)
};

#endif // HYBRID_ASTAR_IPC_SHARED_MEMORY_H
