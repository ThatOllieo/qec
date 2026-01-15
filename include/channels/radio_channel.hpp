#pragma once
#include "../comms_message.hpp"     // CommsMessage, MessageType, ChannelId
#include "ichannel.hpp"             // IChannel base
#include "../../include/tsqueue.hpp"
#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

struct RadioConfig {
    // SPI device
    std::string spidev = "/dev/spidev0.1";
    uint32_t    spi_speed_hz = 2'000'000;

    // GPIO (BCM)
    int pin_reset = 22; // RFM69 RESET
    int pin_dio0  = 25; // RFM69 DIO0 (PayloadReady/PacketSent)

    // RF settings (defaults match your working args example)
    uint32_t freq_hz   = 434'000'000;
    bool     pa_high   = true;      // use +20dBm PA path
    bool     pa_rfio   = false;     // use PA0 (RFIO) instead of PA_BOOST
    int      power_dbm = 17;        // output power (clamped internally)

    int      preamble_bytes = 6;    // demo preamble
    double   rx_bw_khz = 100.0;
    double   bitrate   = 55'000.0;
    double   fdev_hz   = 50'000.0;
};

class RadioChannel : public IChannel {
public:
    explicit RadioChannel(RadioConfig cfg);
    ~RadioChannel() override;

    bool start() override;
    void stop() override;
    void send(const CommsMessage& msg) override;

    // IChannel required callbacks/state accessors
    void set_rx_callback(RxCallback cb) override { rx_cb_ = std::move(cb); }
    ChannelState state() const override { return state_.load(std::memory_order_relaxed); }
    void set_state_callback(StateCallback cb) override { state_cb_ = std::move(cb); }
    ChannelId id() const override { return ChannelId::Radio; }

private:
    // helper to change/notify state
    void set_state(ChannelState s) {
        state_.store(s, std::memory_order_relaxed);
        if (state_cb_) state_cb_(s);
    }

    // --- driver helpers (implemented in .cpp) ---
    bool hw_open();
    void hw_close();
    bool hw_configure();                     // program registers
    bool hw_set_mode(uint8_t mode);          // sleep/stdby/fs/tx/rx
    bool hw_tx_packet(const std::vector<uint8_t>& payload, double timeout_s = 2.0);
    bool hw_rx_once(std::vector<uint8_t>& out, double timeout_s = 0.1);

    // internal threads
    void rx_loop();
    void tx_loop();

    // wire format
    static std::vector<uint8_t> serialize(const CommsMessage& m);
    static bool parse(const std::vector<uint8_t>& f, CommsMessage& out);

private:
    RadioConfig cfg_;

    // runtime
    std::atomic<bool> running_{false};
    std::thread rx_thread_;
    std::thread tx_thread_;

    // tx queue of raw PHY frames
    TSQueue<std::vector<uint8_t>> tx_frames_;

    // low-level state
    int  spi_fd_ = -1;        // spidev fd
    void* gpio_chip_ = nullptr; // gpiod_chip*
    void* gpio_reset_ = nullptr; // gpiod_line*
    void* gpio_dio0_  = nullptr; // gpiod_line*

    // IChannel state + callbacks
    std::atomic<ChannelState>  state_{ChannelState::Stopped};
    RxCallback    rx_cb_;
    StateCallback state_cb_;

    // cached mapped buffer sizes etc. not needed here; FIFO-based
};