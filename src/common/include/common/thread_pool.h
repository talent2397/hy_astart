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
 * @brief 通用线程池 (生产者-消费者模式)
 *
 * 轻量级实现, 用于 CPU 密集型并行任务。
 * Planner 的邻居生成、碰撞检测, Tracker 的参考点搜索均可使用。
 *
 * 设计要点:
 * - 工作线程数 = CPU 核心数 - 1 (留一个给主线程)
 * - 任务队列 + 条件变量通知, 工作线程竞争取任务
 * - 析构时等待所有任务完成再退出
 */
class ThreadPool {
public:
    /**
     * @param num_threads 工作线程数, 0 表示自动 (CPU 核心数 - 1)
     */
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
        condition_.notify_all();             // 唤醒所有等待的工作线程
        for (auto& w : workers_) {
            if (w.joinable()) w.join();
        }
    }

    ThreadPool(const ThreadPool&) = delete;
    ThreadPool& operator=(const ThreadPool&) = delete;

    /**
     * @brief 提交一个任务到线程池
     * @return std::future, 调用方可通过 .get() 获取返回值 (阻塞等待)
     *
     * 使用示例:
     *   auto fut = pool.Submit([](int a, int b) { return a + b; }, 1, 2);
     *   int result = fut.get();  // = 3
     */
    template<typename F, typename... Args>
    auto Submit(F&& f, Args&&... args)
        -> std::future<typename std::result_of<F(Args...)>::type> {
        using ReturnType = typename std::result_of<F(Args...)>::type;

        // 用 packaged_task 包装可调用对象, 通过 future 获取结果
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
        condition_.notify_one();  // 唤醒一个工作线程
        return result;
    }

    /**
     * @brief 并行 for 循环 — 将迭代范围均分给所有工作线程
     *
     * 使用示例:
     *   pool.ParallelFor(0, 100, [&](int i) { data[i] = compute(i); });
     *
     * 如果工作线程数为 0 (单线程), 直接在当前线程串行执行
     */
    template<typename IndexType, typename Func>
    void ParallelFor(IndexType start, IndexType end, Func&& func) {
        if (start >= end) return;
        if (workers_.empty()) {
            // 无工作线程时串行执行
            for (IndexType i = start; i < end; ++i) func(i);
            return;
        }

        IndexType count = end - start;
        size_t num_workers = workers_.size();
        size_t chunk = (count + num_workers - 1) / num_workers;  // 向上取整均分

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

        // 等待所有分块完成
        for (auto& f : futures) f.wait();
    }

    /**
     * @brief 等待所有已提交的任务完成
     *
     * 实现: 提交一个空任务作为"屏障", 当它执行时前面的任务都已处理完
     */
    void WaitAll() {
        auto barrier = Submit([]() {});
        barrier.wait();
    }

    size_t WorkerCount() const { return workers_.size(); }

    /// 当前排队中的任务数 (不含正在执行的)
    size_t PendingTasks() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return tasks_.size();
    }

private:
    /// 工作线程主循环: 阻塞等待任务 → 取出 → 执行 → 重复
    void WorkerLoop() {
        while (true) {
            std::function<void()> task;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                // 条件变量: 等待 stop_ 或 队列非空
                condition_.wait(lock, [this] {
                    return stop_ || !tasks_.empty();
                });
                if (stop_ && tasks_.empty()) return;  // 析构退出
                task = std::move(tasks_.front());
                tasks_.pop();
            }
            task();  // 在锁外执行, 避免阻塞其他线程
        }
    }

    std::vector<std::thread> workers_;              // 工作线程列表
    std::queue<std::function<void()>> tasks_;       // 任务队列
    mutable std::mutex mutex_;                      // 保护任务队列的互斥锁
    std::condition_variable condition_;             // 条件变量 (通知有新任务/停止)
    std::atomic<bool> stop_;                        // 停止标志 (原子变量)
};

#endif // HYBRID_ASTAR_COMMON_THREAD_POOL_H
