#ifndef HYBRID_ASTAR_COMMON_TIMER_H
#define HYBRID_ASTAR_COMMON_TIMER_H

#include <chrono>
#include <string>
#include <unordered_map>

/**
 * @brief 高精度计时器，用于性能统计
 */
class Timer {
public:
    Timer() : start_time_(std::chrono::high_resolution_clock::now()) {}

    // 重置计时器
    void Reset() {
        start_time_ = std::chrono::high_resolution_clock::now();
    }

    // 返回经过的毫秒数
    double End() const {
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time_);
        return static_cast<double>(duration.count()) / 1e6;  // ns → ms
    }

    // 返回经过的微秒数
    double EndMicro() const {
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time_);
        return static_cast<double>(duration.count()) / 1e3;  // ns → μs
    }

private:
    std::chrono::time_point<std::chrono::high_resolution_clock> start_time_;
};

/**
 * @brief 带历史统计的计时器
 */
class StatsTimer {
public:
    StatsTimer(const std::string& name) : name_(name) {}

    void Start() {
        start_ = std::chrono::high_resolution_clock::now();
    }

    void Stop() {
        auto end = std::chrono::high_resolution_clock::now();
        auto dur = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start_).count();
        total_ns_ += dur;
        count_++;
    }

    double AvgMs() const {
        if (count_ == 0) return 0.0;
        return (total_ns_ / static_cast<double>(count_)) / 1e6;
    }

    const std::string& Name() const { return name_; }
    size_t Count() const { return count_; }

private:
    std::string name_;
    std::chrono::time_point<std::chrono::high_resolution_clock> start_;
    uint64_t total_ns_ = 0;
    size_t count_ = 0;
};

#endif // HYBRID_ASTAR_COMMON_TIMER_H
