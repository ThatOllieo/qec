#pragma once
#include <thread>
#include <unordered_map>
#include <memory>
#include "tsqueue.hpp"
#include "events.hpp"
#include "channels/ichannel.hpp"
#include <atomic>
#include <mutex>
#include <chrono>

class CommsManager {
public:
    explicit CommsManager(TSQueue<Event>& main_events);
    ~CommsManager();

    bool start();
    void stop();

    // Register a channel (call before start())
    void register_channel(std::unique_ptr<IChannel> ch);
    void set_src(uint8_t sat_src);

    bool start_channel(ChannelId id);
    void stop_channel(ChannelId id);
    void restart_channel(ChannelId id);

    void enable_autoreconnect(ChannelId id, bool on);

    // Generic send (still available)
    void send(const CommsMessage& msg);

    // Friendly replies 
    void reply_ok_command(uint16_t correlation_id);
    void reply_err_command(uint16_t correlation_id);
    void reply_telem(uint16_t correlation_id, const std::vector<uint8_t>& data);

    uint16_t request_telem_async(
        uint8_t dest,
        ChannelId via,
        uint16_t sensor_id,
        std::chrono::milliseconds timeout,
        int retries
    );

    uint16_t send_command_async(
        uint8_t dest,
        ChannelId via,
        uint16_t command_id,
        const std::vector<uint8_t>& args,
        std::chrono::milliseconds timeout,
        int retries
    );

    //cancel a pending outbound request by correlation id
    void cancel_pending(uint16_t correlation_id);

private:
    TSQueue<Event>& main_events_;
    TSQueue<CommsMessage> inbound_;   // channel -> manager
    TSQueue<CommsMessage> outbound_;  // manager <- main (generic)

    struct ChanWrap{
        std::unique_ptr<IChannel> ch;
        std::atomic<ChannelState> state{ChannelState::Stopped};
        std::atomic<bool> autoreconnect{true};
        std::chrono::milliseconds backoff{500};
        std::chrono::milliseconds backoff_max{5000};
        std::thread rebooter;
    };
    std::unordered_map<ChannelId, ChanWrap> chans_;

    void on_channel_state(ChannelId id, ChannelState st);
    void schedule_reconnect(ChannelId id);

    // ---------------- Pending outbound requests ----------------
    struct PendingEntry {
        MessageType expect_type;                        // TelemetryResp or CommandAck
        CommsMessage original;                          // original request message (for retry)
        std::chrono::steady_clock::time_point deadline; // when to retry/fail
        int retries_left = 0;                           // remaining retries
    };

    std::mutex pending_mx_;
    std::unordered_map<uint16_t, PendingEntry> pending_; // corrId -> entry
    std::atomic<uint16_t> next_corr_{1};

    std::atomic<bool> pending_running_{false};
    std::thread pending_thread_;

    uint16_t reserve_corr();
    void pending_supervisor_loop();

    // Route helper: send using the channel indicated by m.channel_hint
    bool send_via(const CommsMessage& m);

    std::thread inbound_worker_;
    std::thread outbound_worker_;
    std::atomic<bool> running_{false};

    uint8_t sat_src;

    // Channel registry
    std::unordered_map<ChannelId, std::unique_ptr<IChannel>> channels_;

    // Route table for replies: where to send a response for a given corrId
    struct ReplyRoute {
        uint8_t  requester_id;     // original src
        ChannelId via;             // channel to use
    };
    std::unordered_map<uint16_t, ReplyRoute> reply_routes_; // key: correlation_id
    std::mutex routes_mx; // protects reply_routes_

    void inbound_loop();
    void outbound_loop();
    void on_channel_receive(const CommsMessage& m);

    // Map inbound CommsMessage -> semantic Event (Command/TelemetryRequest)
    void raise_semantic_event_and_track_route(const CommsMessage& m);
};
