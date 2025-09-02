#pragma once
#include <string>
#include <thread>
#include <atomic>
#include <vector>
#include <cstdint>
#include <optional>
#include <functional>
#include "ichannel.hpp"
#include "../tsqueue.hpp"

struct TcpConfig {
    std::string host = "127.0.0.1";
    uint16_t    port = 5000;
};

class TcpChannel : public IChannel {
public:
    explicit TcpChannel(TcpConfig cfg);
    ~TcpChannel() override;

    bool start() override;
    void stop() override;

    void send(const CommsMessage& msg) override;   // enqueue and TX thread writes
    void set_rx_callback(RxCallback cb) override { on_receive_ = std::move(cb); }
    ChannelId id() const override { return ChannelId::Wifi; } // still “Wifi” transport

    ChannelState state() const override { return state_.load(); }
    void set_state_callback(StateCallback cb) override { state_cb_ = std::move(cb); }

private:
    TcpConfig cfg_;
    RxCallback on_receive_;
    std::atomic<bool> running_{false};

    int sock_ = -1;
    std::thread rx_thread_;
    std::thread tx_thread_;
    TSQueue<std::vector<uint8_t>> tx_frames_;   // serialized frames ready to send

    // framing
    static std::vector<uint8_t> serialize(const CommsMessage& m);
    static bool parse(const std::vector<uint8_t>& f, CommsMessage& out);

    // socket helpers
    bool connect_once();
    void close_sock();
    bool write_all(const uint8_t* data, size_t n);
    bool read_exact(uint8_t* data, size_t n);

    // loops
    void rx_loop();
    void tx_loop();

    // small endian helpers
    static inline void put16(std::vector<uint8_t>& f, uint16_t v) {
        f.push_back(uint8_t(v & 0xFF)); f.push_back(uint8_t(v >> 8));
    }
    static inline uint16_t get16(const uint8_t* p) {
        return uint16_t(p[0] | (uint16_t(p[1]) << 8));
    }

    std::atomic<ChannelState> state_{ChannelState::Stopped};
    StateCallback state_cb_;

    void set_state(ChannelState st) {
        state_.store(st, std::memory_order_relaxed);
        if (state_cb_) state_cb_(st);
    }
};