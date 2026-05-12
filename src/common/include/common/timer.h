#ifndef HYBRID_ASTAR_COMMON_TIMER_H
#define HYBRID_ASTAR_COMMON_TIMER_H

#include <chrono>
#include <string>
#include <unordered_map>

/**
 * @brief 高精度单次计时器 — 用于测量一段代码的执行时间
 *
 * 使用 std::chrono::high_resolution_clock, 精度可达纳秒级。
 *
 * 使用示例:
 *   Timer t;
 *   doWork();
 *   double ms = t.End();  // 返回经过的毫秒数
 */
class Timer {
public:
    /// 构造时自动记录起始时间
    Timer() : start_time_(std::chrono::high_resolution_clock::now()) {}

    /// 重置起始时间 (用于多次测量)
    void Reset() {
        start_time_ = std::chrono::high_resolution_clock::now();
    }

    /// 返回从构造/Reset 到当前的毫秒数
    double End() const {
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time_);
        return static_cast<double>(duration.count()) / 1e6;  // ns → ms
    }

    /// 返回从构造/Reset 到当前的微秒数
    double EndMicro() const {
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time_);
        return static_cast<double>(duration.count()) / 1e3;  // ns → μs
    }

private:
    std::chrono::time_point<std::chrono::high_resolution_clock> start_time_;
};

/**
 * @brief 统计计时器 — 支持多次 Start/Stop, 记录平均耗时
 *
 * 用于性能分析: 对同一操作多次计时, 统计平均耗时和调用次数。
 *
 * 使用示例:
 *   StatsTimer st("collision_check");
 *   for (int i = 0; i < 1000; ++i) {
 *       st.Start();
 *       CheckCollision(...);
 *       st.Stop();
 *   }
 *   LOG(INFO) << st.Name() << " avg=" << st.AvgMs() << "ms, count=" << st.Count();
 */
class StatsTimer {
public:
    StatsTimer(const std::string& name) : name_(name) {}

    /// 开始计时 (每次调用覆盖上一次 start 时间)
    void Start() {
        start_ = std::chrono::high_resolution_clock::now();
    }

    /// 停止计时, 累加本次耗时
    void Stop() {
        auto end = std::chrono::high_resolution_clock::now();
        auto dur = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start_).count();
        total_ns_ += dur;
        count_++;
    }

    /// 返回平均耗时 (毫秒)
    double AvgMs() const {
        if (count_ == 0) return 0.0;
        return (total_ns_ / static_cast<double>(count_)) / 1e6;
    }

    const std::string& Name() const { return name_; }
    size_t Count() const { return count_; }

private:
    std::string name_;                                                         // 计时器名称 (用于日志输出)
    std::chrono::time_point<std::chrono::high_resolution_clock> start_;       // 本次计时的起始时间
    uint64_t total_ns_ = 0;                                                    // 累计纳秒数
    size_t count_ = 0;                                                         // 计时次数
};

#endif // HYBRID_ASTAR_COMMON_TIMER_H
