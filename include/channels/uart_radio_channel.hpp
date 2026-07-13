#pragma once
#include "../comms_message.hpp"
#include "ichannel.hpp"
#include "../../include/tsqueue.hpp"

#include <atomic>
#include <cstdint>
#include <functional>
#include <string>
#include <thread>
#include <vector>

/**
 * UartRadioChannel
 *
 * Implements IChannel over a hardware UART serial port connecting the OBC
 * (CM4/CM5) to the RP2354 LoRa/FSK bridge board. Replaces RadioChannel's
 * SPI+GPIO hardware layer with a plain POSIX serial fd.
 *
 * The wire format on UART is identical to what the RP2354 bridge expects:
 *
 *   [len : 2 bytes LE][frame bytes...]
 *
 * where 'frame bytes' is exactly what serialize() produces:
 *   [body_len : 2 LE][type : 1][correlation_id : 2 LE][src : 1][dest : 1][command_or_sensor_id : 2 LE][payload...]
 *
 * serialize() and parse() are copied verbatim from RadioChannel — they are
 * transport-agnostic and unchanged. Only the hw_* layer below them is new.
 *
 * Threading model mirrors RadioChannel exactly:
 *   - tx_thread_: blocks on TSQueue::pop(), writes frames to the UART fd
 *   - rx_thread_: blocks on read(), reassembles frames, fires rx_cb_
 *
 * Usage:
 *   UartRadioConfig cfg;
 *   cfg.device   = "/dev/ttyAMA0";
 *   cfg.baud     = 115200;
 *   auto ch = std::make_unique<UartRadioChannel>(cfg);
 *   ch->set_rx_callback([](const CommsMessage& m){ ... });
 *   ch->start();
 *   ch->send(msg);
 */

struct UartRadioConfig {
    std::string device   = "/dev/ttyAMA0";  /* CM4 hardware UART */
    uint32_t    baud     = 115200;
    size_t      max_frame_len = 512;        /* bytes; reject frames larger than this */
};

class UartRadioChannel : public IChannel {
public:
    explicit UartRadioChannel(UartRadioConfig cfg);
    ~UartRadioChannel() override;

    bool start() override;
    void stop()  override;
    bool send(const CommsMessage& msg) override;

    void         set_rx_callback(RxCallback cb)     override { rx_cb_ = std::move(cb); }
    ChannelState state()                      const override { return state_.load(std::memory_order_relaxed); }
    void         set_state_callback(StateCallback cb) override { state_cb_ = std::move(cb); }
    ChannelId    id()                         const override { return ChannelId::Uart; }

private:
    void set_state(ChannelState s) {
        state_.store(s, std::memory_order_relaxed);
        if (state_cb_) state_cb_(s);
    }

    /* UART hw layer */
    bool hw_open();
    void hw_close();

    /* Threads */
    void tx_loop();
    void rx_loop();

    /* Wire format — identical to RadioChannel */
    static std::vector<uint8_t> serialize(const CommsMessage& m);
    static bool                 parse(const std::vector<uint8_t>& f, CommsMessage& out);

    /* Write a complete length-prefixed frame to the fd, retrying on EINTR */
    bool write_frame(const std::vector<uint8_t>& frame);

    UartRadioConfig cfg_;

    int  uart_fd_ = -1;

    std::atomic<bool>          running_{false};
    std::thread                rx_thread_;
    std::thread                tx_thread_;
    TSQueue<std::vector<uint8_t>> tx_frames_;

    std::atomic<ChannelState>  state_{ChannelState::Stopped};
    RxCallback                 rx_cb_;
    StateCallback              state_cb_;
};
