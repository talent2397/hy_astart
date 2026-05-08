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
 * @brief POSIX 共享内存底层封装
 *
 * 使用 shm_open + mmap，支持双缓冲无锁读写。
 * 使用方式：
 *   SharedMemory<MyData> shm;
 *   shm.Create("/my_shm", 0666);    // 创建
 *   shm.Open("/my_shm", false);     // 只读打开
 *   shm->field = value;             // 直接访问
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
    // 创建共享内存段（writer 端调用）
    bool Create(const std::string& name, mode_t mode = 0666) {
        name_ = name;
        is_creator_ = true;

        shm_fd_ = shm_open(name_.c_str(), O_CREAT | O_RDWR, mode);
        if (shm_fd_ < 0) {
            perror("shm_open (create)");
            return false;
        }

        if (ftruncate(shm_fd_, sizeof(T)) < 0) {
            perror("ftruncate");
            return false;
        }

        ptr_ = static_cast<T*>(mmap(nullptr, sizeof(T),
                                    PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_, 0));
        if (ptr_ == MAP_FAILED) {
            perror("mmap");
            return false;
        }

        // 初始化内存区域为零（使用 value-initialization）
        new (ptr_) T{};
        return true;
    }

    // ========== 读取者接口 ==========
    // 打开已存在的共享内存段（reader 端调用）
    bool Open(const std::string& name, bool read_only = true) {
        name_ = name;
        is_creator_ = false;

        int flags = read_only ? O_RDONLY : O_RDWR;
        shm_fd_ = shm_open(name_.c_str(), flags, 0);
        if (shm_fd_ < 0) {
            // 如果打开失败，可能是还没有创建，静默返回 false
            return false;
        }

        int prot = read_only ? PROT_READ : (PROT_READ | PROT_WRITE);
        ptr_ = static_cast<T*>(mmap(nullptr, sizeof(T), prot,
                                    MAP_SHARED, shm_fd_, 0));
        if (ptr_ == MAP_FAILED) {
            perror("mmap");
            return false;
        }

        return true;
    }

    // 关闭并清理
    void Close() {
        if (ptr_ && ptr_ != MAP_FAILED) {
            munmap(ptr_, sizeof(T));
            ptr_ = nullptr;
        }
        if (shm_fd_ >= 0) {
            close(shm_fd_);
            shm_fd_ = -1;
        }
        if (is_creator_ && !name_.empty()) {
            shm_unlink(name_.c_str());
        }
    }

    // 只清理共享内存段（不关闭 fd/mmap）
    static void Unlink(const std::string& name) {
        shm_unlink(name.c_str());
    }

    // 指针访问
    T* operator->() { return ptr_; }
    const T* operator->() const { return ptr_; }
    T& operator*() { return *ptr_; }
    const T& operator*() const { return *ptr_; }
    T* Get() { return ptr_; }
    const T* Get() const { return ptr_; }

    bool IsValid() const { return ptr_ != nullptr && ptr_ != MAP_FAILED; }
    const std::string& Name() const { return name_; }

private:
    std::string name_;
    int shm_fd_ = -1;
    T* ptr_ = nullptr;
    bool is_creator_ = false;
};

#endif // HYBRID_ASTAR_IPC_SHARED_MEMORY_H
