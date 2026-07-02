#pragma once
#include <cstdint>
#include <vector>
#include "message_types.hpp"

enum class ChannelId : uint8_t {
    Uart  = 0,
    Radio = 1,
    Wifi  = 2,
    Can   = 3,
    Auto  = 255
};

struct CommsMessage {
    uint8_t     src = 0;
    uint8_t     dest = 0;
    MessageType type = MessageType::I_CMD_RQ;
    uint16_t    correlation_id = 0;
    ChannelId   channel_hint = ChannelId::Auto;
    std::vector<uint8_t> payload;   // opaque to CommsManager
};
