#include <thread>
#include <chrono>
#include <functional>
#include <cstring>

// fire-and-forget timeout helper, made to be like the one you get in javascript
inline void setTimeout(std::function<void()> fn, std::chrono::milliseconds delay) {
    std::thread([fn, delay]() {
        std::this_thread::sleep_for(delay);
        fn();
    }).detach();
}

template <typename T>
std::array<uint8_t, sizeof(T)> toBytes(const T &value) {
    std::array<uint8_t, sizeof(T)> out{};
    std::memcpy(out.data(), &value, sizeof(T));
    return out;
}

template <typename T>
void appendBytes(std::vector<uint8_t>& buf, const T &value) {
    auto arr = toBytes(value);
    buf.insert(buf.end(), arr.begin(), arr.end());
}