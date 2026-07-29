#pragma once
#include <condition_variable>
#include <mutex>
#include <queue>

//thread safe queue, used for the main event list and also some other bits
template <typename T>
class TSQueue {
public:
    void push(T v) {
        { std::lock_guard<std::mutex> lk(m_); q_.push(std::move(v)); }
        cv_.notify_one();
    }

    T pop() {
        std::unique_lock<std::mutex> lk(m_);
        cv_.wait(lk, [&]{ return !q_.empty(); });
        T v = std::move(q_.front()); q_.pop(); return v;
    }

    bool try_pop(T& out) {
        std::lock_guard<std::mutex> lk(m_);
        if (q_.empty()) return false;
        out = std::move(q_.front()); q_.pop(); return true;
    }

private:
    std::mutex m_;
    std::condition_variable cv_;
    std::queue<T> q_;
};
