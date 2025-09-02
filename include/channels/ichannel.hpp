#pragma once
#include <functional>
#include "../comms_message.hpp"

enum class ChannelState : uint8_t {Stopped, Starting, Running, Failed };

class IChannel {
public:
    virtual ~IChannel() = default;

    virtual bool start() = 0;
    virtual void stop() = 0;

    // CommsManager calls this to transmit a logical message; channel serializes it
    virtual void send(const CommsMessage& msg) = 0;

    // CommsManager sets this callback; channel calls it when it reassembles a message
    using RxCallback = std::function<void(const CommsMessage&)>;
    virtual void set_rx_callback(RxCallback cb) = 0;


    virtual ChannelState state() const = 0;

    using StateCallback = std::function<void(ChannelState)>;
    virtual void set_state_callback(StateCallback cb) = 0;

    
    // Identity
    virtual ChannelId id() const = 0;
};
