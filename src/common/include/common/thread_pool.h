#ifndef HYBRID_ASTAR_COMMON_THREAD_POOL_H
#define HYBRID_ASTAR_COMMON_THREAD_POOL_H

#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <future>
#include <functional>
#include <atomic>
#include <memory>

/**
 * @brief 通用线程池
 *
 * 轻量级实现，用于 CPU 密集型并行任务。
 * Planner 的邻居生成、碰撞检测，Tracker 的参考点搜索均可使用。
 */
class ThreadPool {
public:
    explicit ThreadPool(size_t num_threads = 0) : stop_(false) {
        if (num_threads == 0) {
            num_threads = std::thread::hardware_concurrency();
            if (num_threads == 0) num_threads = 4;
        }
        if (num_threads > 1) num_threads--;  // 留一个给主线程

        workers_.reserve(num_threads);
        for (size_t i = 0; i < num_threads; ++i) {
            workers_.emplace_back([this] { WorkerLoop(); });
        }
    }

    ~ThreadPool() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stop_ = true;
        }
        condition_.notify_all();
        for (auto& w : workers_) {
            if (w.joinable()) w.join();
        }
    }

    ThreadPool(const ThreadPool&) = delete;
    ThreadPool& operator=(const ThreadPool&) = delete;

    // 提交任务，返回 future
    template<typename F, typename... Args>
    auto Submit(F&& f, Args&&... args)
        -> std::future<typename std::result_of<F(Args...)>::type> {
        using ReturnType = typename std::result_of<F(Args...)>::type;

        auto task = std::make_shared<std::packaged_task<ReturnType()>>(
            std::bind(std::forward<F>(f), std::forward<Args>(args)...)
        );

        std::future<ReturnType> result = task->get_future();
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (stop_) {
                throw std::runtime_error("ThreadPool is stopped");
            }
            tasks_.emplace([task]() { (*task)(); });
        }
        condition_.notify_one();
        return result;
    }

    // 并行执行 for 循环
    // func(idx) 会被多个线程调用
    template<typename IndexType, typename Func>
    void ParallelFor(IndexType start, IndexType end, Func&& func) {
        if (start >= end) return;
        if (workers_.empty()) {
            for (IndexType i = start; i < end; ++i) func(i);
            return;
        }

        IndexType count = end - start;
        size_t num_workers = workers_.size();
        size_t chunk = (count + num_workers - 1) / num_workers;

        std::vector<std::future<void>> futures;
        for (size_t w = 0; w < num_workers; ++w) {
            IndexType w_start = start + w * chunk;
            IndexType w_end = std::min(w_start + chunk, end);
            if (w_start >= w_end) break;

            futures.push_back(Submit([&func, w_start, w_end]() {
                for (IndexType i = w_start; i < w_end; ++i) {
                    func(i);
                }
            }));
        }

        for (auto& f : futures) f.wait();
    }

    // 等待所有排队任务完成
    void WaitAll() {
        // 提交一个屏障任务
        auto barrier = Submit([]() {});
        barrier.wait();
    }

    size_t WorkerCount() const { return workers_.size(); }
    size_t PendingTasks() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return tasks_.size();
    }

private:
    void WorkerLoop() {
        while (true) {
            std::function<void()> task;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                condition_.wait(lock, [this] {
                    return stop_ || !tasks_.empty();
                });
                if (stop_ && tasks_.empty()) return;
                task = std::move(tasks_.front());
                tasks_.pop();
            }
            task();
        }
    }

    std::vector<std::thread> workers_;
    std::queue<std::function<void()>> tasks_;
    mutable std::mutex mutex_;
    std::condition_variable condition_;
    std::atomic<bool> stop_;
};

#endif // HYBRID_ASTAR_COMMON_THREAD_POOL_H
