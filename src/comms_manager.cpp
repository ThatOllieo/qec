#include "../include/comms_manager.hpp"
#include "../include/message_types.hpp"
#include <chrono>
#include <iostream>

//Constructor
CommsManager::CommsManager(TSQueue<Event>& main_events)
    : main_events_(main_events) {}

//destructor
CommsManager::~CommsManager() { stop(); }

//sets up new channel and binds said channels callbacks to functions within comms manager, to reciveve
void CommsManager::register_channel(std::unique_ptr<IChannel> ch) {
    ChannelId cid = ch->id();
    // Bind callbacks once when registering
    ch->set_rx_callback([this](const CommsMessage& m){ on_channel_receive(m); });
    ch->set_state_callback([this, cid](ChannelState st){ on_channel_state(cid, st); });

    std::lock_guard<std::mutex> lk(chans_mx_);
    auto& cw = chans_[cid];
    cw.ch = std::move(ch);
    cw.state.store(ChannelState::Stopped, std::memory_order_relaxed);
    cw.autoreconnect.store(true, std::memory_order_relaxed);
    cw.backoff     = std::chrono::milliseconds(500);
    cw.backoff_max = std::chrono::milliseconds(5000);
}

//sets the devices comms identifier
void CommsManager::set_src(uint8_t ss){
    sat_src = ss;
}

//if not already running, starts comms management threads
bool CommsManager::start() {
    if (running_) return true;

    std::vector<IChannel*> channels_to_start;
    {
        std::lock_guard<std::mutex> lk(chans_mx_);
        if (chans_.empty()) {
            std::cout << "NO REG CHAN" << std::endl;
            return false;
        }
        channels_to_start.reserve(chans_.size());
        for (auto& [cid, cw] : chans_) {
            channels_to_start.push_back(cw.ch.get());
        }
    }

    running_ = true;
    try {
        for (IChannel* ch : channels_to_start) {
            if (ch) ch->start();
        }

        inbound_worker_  = std::thread(&CommsManager::inbound_loop, this);
        outbound_worker_ = std::thread(&CommsManager::outbound_loop, this);

        pending_running_ = true;
        pending_thread_ = std::thread(&CommsManager::pending_supervisor_loop, this);

        return true;
    } catch (...) {
        running_ = false;
        pending_running_ = false;
        stop();
        throw;
    }
}

//if running, stop the threads and shuts down channels.
void CommsManager::stop() {
    if (!running_ && !pending_running_) return;

    running_ = false;

    outbound_.push(CommsMessage{});
    inbound_.push(CommsMessage{});

    pending_running_ = false;
    if (pending_thread_.joinable()) pending_thread_.join();

    if (inbound_worker_.joinable())  inbound_worker_.join();
    if (outbound_worker_.joinable()) outbound_worker_.join();

    struct ChannelStopTarget {
        ChannelId id;
        IChannel* ch;
        std::thread* rebooter;
    };

    std::vector<ChannelStopTarget> channels_to_stop;
    {
        std::lock_guard<std::mutex> lk(chans_mx_);
        channels_to_stop.reserve(chans_.size());
        for (auto& [cid, cw] : chans_) {
            cw.autoreconnect = false;
            channels_to_stop.push_back(ChannelStopTarget{cid, cw.ch.get(), &cw.rebooter});
        }
    }

    for (auto& target : channels_to_stop) {
        if (target.rebooter && target.rebooter->joinable()) target.rebooter->join();
        if (target.ch) target.ch->stop();
    }

    {
        std::lock_guard<std::mutex> lk(chans_mx_);
        for (auto& [cid, cw] : chans_) {
            cw.state = ChannelState::Stopped;
        }
        chans_.clear();
    }

    reply_routes_.clear();
}

//low level pushes the CommsMessage it recieves to the outbound queue
void CommsManager::send(const CommsMessage& msg) {
    outbound_.push(msg);
}

//loops through the inbound queue and processes each message
void CommsManager::inbound_loop() {
    while (running_) {
        try {
            CommsMessage in = inbound_.pop();
            if (!running_) break;
            raise_semantic_event_and_track_route(in);
        } catch (const CommsError& ex) {
            std::cerr << severityTag(ex.severity()) << "[COMMS] inbound_loop error: " << ex.what() << "\n";
            Event e; e.type = EventType::ModuleFailed;
            e.data = EvModuleFailed{"inbound_worker", ex.what(), ex.severity()};
            main_events_.push(std::move(e));
        } catch (const std::exception& ex) {
            std::cerr << "[ERROR][COMMS] inbound_loop error: " << ex.what() << "\n";
            Event e; e.type = EventType::ModuleFailed;
            e.data = EvModuleFailed{"inbound_worker", ex.what(), ErrorSeverity::Recoverable};
            main_events_.push(std::move(e));
        } catch (...) {
            std::cerr << "[ERROR][COMMS] inbound_loop unknown error\n";
            Event e; e.type = EventType::ModuleFailed;
            e.data = EvModuleFailed{"inbound_worker", "unknown exception", ErrorSeverity::Recoverable};
            main_events_.push(std::move(e));
        }
    }
}

//loops through commsMessages in the outbound queue, finds the best ChannelId to send it on and passes it to that channel
void CommsManager::outbound_loop() {
    while (running_) {
        try {
            CommsMessage out = outbound_.pop();
            if (!running_) break;
            ChannelId via = (out.channel_hint == ChannelId::Auto) ? ChannelId::Wifi : out.channel_hint;

            IChannel* ch = nullptr;
            {
                std::lock_guard<std::mutex> lk(chans_mx_);
                auto it = chans_.find(via);
                if (it != chans_.end()) {
                    ch = it->second.ch.get();
                }
            }
            if (ch) {
                if (!ch->send(out)) {
                    std::cerr << "[WARN][COMMS] send() rejected message on channel "
                              << static_cast<int>(via) << " (frame too large or channel not ready)\n";
                }
            } else {
                std::cerr << "[WARN][COMMS] No channel registered for ChannelId="
                          << static_cast<int>(via) << ", message dropped\n";
            }
        } catch (const CommsError& ex) {
            std::cerr << severityTag(ex.severity()) << "[COMMS] outbound_loop error: " << ex.what() << "\n";
            Event e; e.type = EventType::ModuleFailed;
            e.data = EvModuleFailed{"outbound_worker", ex.what(), ex.severity()};
            main_events_.push(std::move(e));
        } catch (const std::exception& ex) {
            std::cerr << "[ERROR][COMMS] outbound_loop error: " << ex.what() << "\n";
            Event e; e.type = EventType::ModuleFailed;
            e.data = EvModuleFailed{"outbound_worker", ex.what(), ErrorSeverity::Recoverable};
            main_events_.push(std::move(e));
        } catch (...) {
            std::cerr << "[ERROR][COMMS] outbound_loop unknown error\n";
            Event e; e.type = EventType::ModuleFailed;
            e.data = EvModuleFailed{"outbound_worker", "unknown exception", ErrorSeverity::Recoverable};
            main_events_.push(std::move(e));
        }
    }
}

//on recieving a new message from a channel, push to the inbound queue
void CommsManager::on_channel_receive(const CommsMessage& m) {
    inbound_.push(m);
}

//used to boot up each individual channel 
bool CommsManager::start_channel(ChannelId id) {
    IChannel* ch = nullptr;
    {
        std::lock_guard<std::mutex> lk(chans_mx_);
        auto it = chans_.find(id);
        if (it == chans_.end()) return false;
        ch = it->second.ch.get();
    }
    return ch ? ch->start() : false;
}

//used to shut down each individual channel
void CommsManager::stop_channel(ChannelId id) {
    IChannel* ch = nullptr;
    std::thread* rebooter = nullptr;
    {
        std::lock_guard<std::mutex> lk(chans_mx_);
        auto it = chans_.find(id);
        if (it == chans_.end()) return;
        auto& cw = it->second;
        cw.autoreconnect = false;
        ch = cw.ch.get();
        rebooter = &cw.rebooter;
    }

    if (rebooter && rebooter->joinable()) rebooter->join();
    if (ch) ch->stop();

    {
        std::lock_guard<std::mutex> lk(chans_mx_);
        auto it = chans_.find(id);
        if (it != chans_.end()) {
            it->second.state = ChannelState::Stopped;
        }
    }
}

//used to restart individual channel eg (restart_channel(ChannelID::Wifi))
void CommsManager::restart_channel(ChannelId id) {
    stop_channel(id);
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    start_channel(id);
}

//used to enable or disable the automatic restart when a channel fails out
void CommsManager::enable_autoreconnect(ChannelId id, bool on) {
    std::thread* rebooter = nullptr;
    {
        std::lock_guard<std::mutex> lk(chans_mx_);
        auto it = chans_.find(id);
        if (it == chans_.end()) return;
        auto& cw = it->second;
        cw.autoreconnect = on;
        if (!on) rebooter = &cw.rebooter;
    }

    if (rebooter && rebooter->joinable()) rebooter->join();
}

//used to track the state of the channels, available states are Stopped, Starting, Running, Failed 
void CommsManager::on_channel_state(ChannelId id, ChannelState st) {
    std::lock_guard<std::mutex> lk(chans_mx_);
    auto it = chans_.find(id);
    if (it == chans_.end()) return;
    auto& cw = it->second;
    cw.state = st;

    // Optional: push an event up so main can log/react (requires EvChannelState/EventType::ChannelStateChanged)
    // Event ev; ev.type = EventType::ChannelStateChanged; ev.data = EvChannelState{ id, st }; main_events_.push(std::move(ev));

    if (st == ChannelState::Failed && cw.autoreconnect) {
        // schedule a reconnect with backoff
        if (!cw.rebooter.joinable()) {
            cw.rebooter = std::thread([this, id]{ schedule_reconnect(id); });
        }
    } else if (st == ChannelState::Running) {
        cw.backoff = std::chrono::milliseconds(500); // reset backoff on success
    }
}

// used to add a delay before trying to reconnect
void CommsManager::schedule_reconnect(ChannelId id) {
    for (;;) {
        if (!running_) return; // CommsManager is shutting down

        try {
            std::chrono::milliseconds backoff{0};
            IChannel *ch = nullptr;
            {
                std::lock_guard<std::mutex> lk(chans_mx_);
                auto it = chans_.find(id);
                if (it == chans_.end()) return; // channel removed

                auto& cw = it->second;
                if (!cw.autoreconnect) return;
                if (cw.state == ChannelState::Running) return;

                backoff = cw.backoff;
                ch = cw.ch.get();
            }

            std::this_thread::sleep_for(backoff);
            if (!running_) return; // check again after sleep — may have slept for up to backoff_max
            if (!ch) return;

            ch->stop();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            if (!running_) return;
            ch->start();

            {
                std::lock_guard<std::mutex> lk(chans_mx_);
                auto it = chans_.find(id);
                if (it == chans_.end()) return;

                auto& cw = it->second;
                cw.backoff = std::min(cw.backoff * 2, cw.backoff_max);

                if (!cw.autoreconnect) return;
                if (cw.state == ChannelState::Running) return;
            }
        } catch (const CommsError& ex) {
            std::cerr << severityTag(ex.severity()) << "[COMMS] Reconnect for channel "
                      << static_cast<int>(id) << " error: " << ex.what() << "\n";
        } catch (const std::exception& ex) {
            std::cerr << "[ERROR][COMMS] Reconnect for channel "
                      << static_cast<int>(id) << " error: " << ex.what() << "\n";
        } catch (...) {
            std::cerr << "[ERROR][COMMS] Reconnect for channel "
                      << static_cast<int>(id) << " unknown error\n";
        }
    }
}

// creates a new correlation id for a new telem request or command send
uint16_t CommsManager::reserve_corr() {
    uint16_t id = next_corr_.fetch_add(1, std::memory_order_relaxed);
    if (id == 0) id = next_corr_.fetch_add(1, std::memory_order_relaxed);
    return id;
}


bool CommsManager::send_via(const CommsMessage& m) {
    if (!running_) return false;
    outbound_.push(m);
    return true;
}

void CommsManager::cancel_pending(uint16_t correlation_id) {
    std::lock_guard<std::mutex> lk(pending_mx_);
    pending_.erase(correlation_id);
}

// Little-endian write helper (kept local here to avoid header dependency)
static inline void put16_le(std::vector<uint8_t>& v, uint16_t x) {
    v.push_back(uint8_t(x & 0xFF));
    v.push_back(uint8_t(x >> 8));
}

uint16_t CommsManager::request_telem_async(uint8_t dest,
                                           ChannelId via,
                                           uint16_t sensor_id,
                                           std::chrono::milliseconds timeout,
                                           int retries) {
    uint16_t corr = reserve_corr();

    CommsMessage m;
    m.type = MessageType::I_TLM_RQ;
    m.correlation_id = corr;
    m.src  = sat_src;
    m.dest = dest;
    m.channel_hint = via;
    m.payload.clear();
    put16_le(m.payload, sensor_id);

    if (!send_via(m)) return 0;

    PendingEntry pe;
    pe.expect_type  = MessageType::I_TLM_PT; // expecting telemetry packet back
    pe.original     = m;
    pe.deadline     = std::chrono::steady_clock::now() + timeout;
    pe.retries_left = retries;

    {
        std::lock_guard<std::mutex> lk(pending_mx_);
        pending_.emplace(corr, std::move(pe));
    }
    return corr;
}

uint16_t CommsManager::send_command_async(uint8_t dest,
                                          ChannelId via,
                                          uint16_t command_id,
                                          const std::vector<uint8_t>& args,
                                          std::chrono::milliseconds timeout,
                                          int retries) {
    uint16_t corr = reserve_corr();

    CommsMessage m;
    m.type = MessageType::I_CMD_RQ;
    m.correlation_id = corr;
    m.src  = sat_src;
    m.dest = dest;
    m.channel_hint = via;
    m.payload.clear();
    put16_le(m.payload, command_id);
    m.payload.insert(m.payload.end(), args.begin(), args.end());

    if (!send_via(m)) return 0;

    PendingEntry pe;
    pe.expect_type  = MessageType::I_CMD_OK; // accept OK as default; errors handled below
    pe.original     = m;
    pe.deadline     = std::chrono::steady_clock::now() + timeout;
    pe.retries_left = retries;

    {
        std::lock_guard<std::mutex> lk(pending_mx_);
        pending_.emplace(corr, std::move(pe));
    }
    return corr;
}

void CommsManager::pending_supervisor_loop() {
    using namespace std::chrono;
    while (pending_running_) {
        try {
            std::this_thread::sleep_for(50ms);
            std::vector<std::pair<uint16_t, PendingEntry>> expired;
            {
                std::lock_guard<std::mutex> lk(pending_mx_);
                auto now = steady_clock::now();
                for (auto it = pending_.begin(); it != pending_.end(); ) {
                    if (now >= it->second.deadline) {
                        expired.emplace_back(it->first, std::move(it->second));
                        it = pending_.erase(it);
                    } else {
                        ++it;
                    }
                }
            }
            for (auto& kv : expired) {
                auto corr = kv.first;
                auto pe   = std::move(kv.second);
                if (pe.retries_left > 0) {
                    CommsMessage resend = pe.original;
                    outbound_.push(resend);
                    pe.deadline = steady_clock::now() + 500ms;
                    pe.retries_left--;
                    std::lock_guard<std::mutex> lk(pending_mx_);
                    pending_.emplace(corr, std::move(pe));
                } else {
                    if (pe.expect_type == MessageType::I_TLM_PT) {
                        EvTelemetryFailed d; d.correlation_id = corr; d.reason = "timeout";
                        Event e; e.type = EventType::TelemetryFailed; e.data = std::move(d);
                        main_events_.push(std::move(e));
                    } else {
                        EvCommandFailed d; d.correlation_id = corr; d.reason = "timeout";
                        Event e; e.type = EventType::CommandFailed; e.data = std::move(d);
                        main_events_.push(std::move(e));
                    }
                }
            }
        } catch (const CommsError& ex) {
            std::cerr << severityTag(ex.severity()) << "[COMMS] pending_supervisor_loop error: " << ex.what() << "\n";
            Event e; e.type = EventType::ModuleFailed;
            e.data = EvModuleFailed{"pending_supervisor", ex.what(), ex.severity()};
            main_events_.push(std::move(e));
        } catch (const std::exception& ex) {
            std::cerr << "[ERROR][COMMS] pending_supervisor_loop error: " << ex.what() << "\n";
            Event e; e.type = EventType::ModuleFailed;
            e.data = EvModuleFailed{"pending_supervisor", ex.what(), ErrorSeverity::Recoverable};
            main_events_.push(std::move(e));
        } catch (...) {
            std::cerr << "[ERROR][COMMS] pending_supervisor_loop unknown error\n";
            Event e; e.type = EventType::ModuleFailed;
            e.data = EvModuleFailed{"pending_supervisor", "unknown exception", ErrorSeverity::Recoverable};
            main_events_.push(std::move(e));
        }
    }
}

// Turn inbound messages into semantic events and remember where to reply
void CommsManager::raise_semantic_event_and_track_route(const CommsMessage& m) {
    // First: is this a reply to one of our outbound requests?
    if (m.type == MessageType::I_TLM_PT || m.type == MessageType::I_CMD_OK || m.type == MessageType::I_CMD_ER) {
        std::unique_lock<std::mutex> lk(pending_mx_);
        auto it = pending_.find(m.correlation_id);
        if (it != pending_.end()) {
            auto pe = std::move(it->second);
            pending_.erase(it);
            lk.unlock();

            // Telemetry response
            if (m.type == MessageType::I_TLM_PT) {
                EvTelemetryArrived d; d.correlation_id = m.correlation_id; d.bytes = m.payload;
                Event e; e.type = EventType::TelemetryArrived; e.data = std::move(d);
                main_events_.push(std::move(e));
                return; // handled
            }
            // Command ack (OK or ER)
            if (m.type == MessageType::I_CMD_OK) {
                EvCommandAcked d; d.correlation_id = m.correlation_id;
                Event e; e.type = EventType::CommandAcked; e.data = std::move(d);
                main_events_.push(std::move(e));
                return;
            } else { // I_CMD_ER
                EvCommandFailed d; d.correlation_id = m.correlation_id; d.reason = "error";
                Event e; e.type = EventType::CommandFailed; e.data = std::move(d);
                main_events_.push(std::move(e));
                return;
            }
        }
        // If it's not pending, fall through (could be unsolicited / late)
    }

    // Track reply route only for request types
    if (m.correlation_id != 0 &&
        (m.type == MessageType::I_CMD_RQ || m.type == MessageType::I_TLM_RQ)) {

        ReplyRoute rr;
        rr.requester_id = m.src;
        rr.via          = m.channel_hint;
        {
            std::lock_guard<std::mutex> lk(routes_mx);
            reply_routes_[m.correlation_id] = rr;
        }
    }

    switch (m.type) {
        case MessageType::I_CMD_RQ: {
            // payload: [cmd_id(2)][args...]
            if (m.payload.size() < 2) {
                std::cerr << "[WARN][COMMS] Dropping malformed command request\n";
                break;
            }
            uint16_t cmd_id = uint16_t(m.payload[0] | (uint16_t(m.payload[1]) << 8));

            EvCommand evp;
            evp.command_id     = cmd_id;
            evp.correlation_id = m.correlation_id;
            evp.requester_id   = m.src;
            evp.reply_via      = static_cast<uint8_t>(m.channel_hint);
            if (m.payload.size() > 2)
                evp.args.assign(m.payload.begin()+2, m.payload.end());

            Event e; e.type = EventType::Command; e.data = std::move(evp);
            main_events_.push(std::move(e));
            break;
        }
        case MessageType::I_TLM_RQ: {
            // payload: [sensor_id(2)][params...]
            if (m.payload.size() < 2) {
                std::cerr << "[WARN][COMMS] Dropping malformed telemetry request\n";
                break;
            }
            uint16_t sensor_id = uint16_t(m.payload[0] | (uint16_t(m.payload[1]) << 8));

            EvTelemetryRequest evp;
            evp.sensor_id      = sensor_id;
            evp.correlation_id = m.correlation_id;
            evp.requester_id   = m.src;
            evp.reply_via      = static_cast<uint8_t>(m.channel_hint);
            if (m.payload.size() > 2)
                evp.params.assign(m.payload.begin()+2, m.payload.end());

            Event e; e.type = EventType::TelemetryRequest; e.data = std::move(evp);
            main_events_.push(std::move(e));
            break;
        }
        default: {
            //Event e; e.type = EventType::CommsInbound; e.data = EvCommsInbound{ m };
            //main_events_.push(std::move(e));
            break;
        }
    }
}

// ---------- Friendly replies (corrId-only) ----------
void CommsManager::reply_ok_command(uint16_t correlation_id) {
    ReplyRoute route;
    {
        std::lock_guard<std::mutex> lk(routes_mx);
        auto it = reply_routes_.find(correlation_id);
        if (it == reply_routes_.end()) return; // unknown or already answered
        route = it->second;
        reply_routes_.erase(it);
    }

    CommsMessage m;
    m.src            = sat_src; // SAT ID (set yours)
    m.dest           = route.requester_id;
    m.type           = MessageType::I_CMD_OK;
    m.correlation_id = correlation_id;
    m.channel_hint   = route.via;
    m.payload = {}; // optional status code

    send(m);
}

void CommsManager::reply_err_command(uint16_t correlation_id) {
    ReplyRoute route;
    {
        std::lock_guard<std::mutex> lk(routes_mx);
        auto it = reply_routes_.find(correlation_id);
        if (it == reply_routes_.end()) return; // unknown or already answered
        route = it->second;
        reply_routes_.erase(it);
    }

    CommsMessage m;
    m.src            = sat_src; // SAT ID (set yours)
    m.dest           = route.requester_id;
    m.type           = MessageType::I_CMD_ER;
    m.correlation_id = correlation_id;
    m.channel_hint   = route.via;
    m.payload = {}; // optional status code

    send(m);
}

void CommsManager::reply_telem(uint16_t correlation_id, const std::vector<uint8_t>& data) {
    ReplyRoute route;
    {
        std::lock_guard<std::mutex> lk(routes_mx);
        auto it = reply_routes_.find(correlation_id);
        if (it == reply_routes_.end()) return;
        route = it->second;
        reply_routes_.erase(it);
    }

    CommsMessage m;
    m.src            = sat_src; // SAT ID
    m.dest           = route.requester_id;
    m.type           = MessageType::I_TLM_PT;
    m.correlation_id = correlation_id;
    m.channel_hint   = route.via;
    m.payload        = data;

    send(m);
}
