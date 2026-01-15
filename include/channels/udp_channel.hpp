#pragma once
#include <thread>
#include <atomic>
#include <vector>
#include <cstdint>
#include <functional>
#include <array>

// POSIX address structs
#include <netinet/in.h>

#include "ichannel.hpp"
#include "../tsqueue.hpp"

struct UdpConfig {
    // Reserved for future options (e.g., subnet/base IP). Currently unused.
};

class UdpChannel : public IChannel {
public:
    explicit UdpChannel(UdpConfig cfg);
    ~UdpChannel() override;

    bool start() override;
    void stop() override;

    void send(const CommsMessage& msg) override;   // enqueue and TX thread writes
    void set_rx_callback(RxCallback cb) override { on_receive_ = std::move(cb); }
    ChannelId id() const override { return ChannelId::Wifi; } // still “Wifi” transport

    ChannelState state() const override { return state_.load(); }
    void set_state_callback(StateCallback cb) override { state_cb_ = std::move(cb); }

private:
    UdpConfig cfg_;
    RxCallback on_receive_;
    std::atomic<bool> running_{false};

    int sock_ = -1;
    std::thread rx_thread_;
    std::thread tx_thread_;
    struct TxItem {
        std::vector<uint8_t> frame;
        sockaddr_in dest{};
    };
    TSQueue<TxItem> tx_items_;   // serialized frames + destination address
    std::array<sockaddr_in, 256> dest_addrs_{}; // 10.42.0.X:5000 indexed by dest ID

    // framing
    static std::vector<uint8_t> serialize(const CommsMessage& m);
    static bool parse(const std::vector<uint8_t>& f, CommsMessage& out);

    // socket helpers
    bool open_and_bind();
    void close_sock();
    void init_dest_table();

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