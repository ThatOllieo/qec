#pragma once
#include <cstdint>

//different types of message that the communication manager can handle
enum class MessageType : uint8_t{
    I_BRD    = 0x00, // Simple

    I_CMD_RQ = 0x11, // Sends command
    I_CMD_OK = 0x01, // Command ack
    I_CMD_ER = 0x03, // Command error

    I_TLM_RQ = 0x12, // Telemetry request
    I_TLM_PT = 0x02, // Telemetry return
    I_TLM_ER = 0x13, // Telemetry error

    I_BIN_RQ = 0x1A, // Request block of data (file)
    I_BIN_PT = 0x0A, // Wants to send a block of data (file)
    I_BIN_DT = 0x0B, // Bin data
    I_BIN_NX = 0x0C, // Next data (continue message)
    I_BIN_RP = 0x1C, // Repeat data (repeat last block of messages)
    I_BIN_AB = 0x1B, // Abort transfer
    I_BIN_DN = 0x0D, // Transfer done (ok)
    I_BIN_ER = 0x1D, // Transfer error
};