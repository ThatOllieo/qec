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

#include <linux/can.h>

struct CanConfig{
    std::string interface;
    uint8_t     sat_src;
};

class CanChannel : public IChannel {
    public:
        explicit CanChannel(CanConfig cfg);
        ~CanChannel() override;

        bool start() override;
        void stop()  override;
        bool send(const CommsMessage& msg) override;

        void         set_rx_callback(RxCallback cb)     override { rx_cb_ = std::move(cb); }
        ChannelState state()                      const override { return state_.load(std::memory_order_relaxed); }
        void         set_state_callback(StateCallback cb) override { state_cb_ = std::move(cb); }
        ChannelId    id()                         const override { return ChannelId::Can; }

    private:
        void set_state(ChannelState s) {
            state_.store(s, std::memory_order_relaxed);
            if (state_cb_) state_cb_(s);
        }

        bool hw_open();
        void hw_close();

        void tx_loop();
        void rx_loop();

        bool serialize(struct can_frame& frame, const CommsMessage& msg) const;
        bool parse(struct can_frame& frame, CommsMessage& msg) const;

        bool write_frame(const struct can_frame& frame);

        CanConfig cfg_;
        int  can_fd_ = -1;

        std::atomic<bool>          running_{false};
        std::thread                rx_thread_;
        std::thread                tx_thread_;
        TSQueue<struct can_frame> tx_frames_;

        std::atomic<ChannelState>  state_{ChannelState::Stopped};
        RxCallback                 rx_cb_;
        StateCallback              state_cb_;


        bool read_frame(struct can_frame& frame);

};