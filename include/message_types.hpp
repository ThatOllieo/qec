#pragma once
#include <cstdint>
#include <map>

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

enum class SSICanMessageType : uint8_t{
    I_BRD    = 0x0, // Simple

    I_CMD_RQ = 0x1, // Sends command
    I_CMD_OK = 0x2, // Command ack
    I_CMD_ER = 0x3, // Command error

    I_TLM_RQ = 0x4, // Telemetry request
    I_TLM_PT = 0x5, // Telemetry return
    I_TLM_ER = 0x6, // Telemetry error

    I_BIN_RQ = 0x7, // Request block of data (file)
    I_BIN_PT = 0x8, // Wants to send a block of data (file)
    I_BIN_DT = 0x9, // Bin data
    I_BIN_NX = 0xA, // Next data (continue message)
    I_BIN_RP = 0xB, // Repeat data (repeat last block of messages)
    I_BIN_AB = 0xC, // Abort transfer
    I_BIN_DN = 0xD, // Transfer done (ok)
    I_BIN_ER = 0x0, // Transfer error
};

inline std::map<MessageType, SSICanMessageType> MessageTypeToSSICanMessageType = {
    {MessageType::I_BRD, SSICanMessageType::I_BRD},
    {MessageType::I_CMD_RQ, SSICanMessageType::I_CMD_RQ},
    {MessageType::I_CMD_OK, SSICanMessageType::I_CMD_OK},
    {MessageType::I_CMD_ER, SSICanMessageType::I_CMD_ER},
    {MessageType::I_TLM_RQ, SSICanMessageType::I_TLM_RQ},
    {MessageType::I_TLM_PT, SSICanMessageType::I_TLM_PT},
    {MessageType::I_TLM_ER, SSICanMessageType::I_TLM_ER},
    {MessageType::I_BIN_RQ, SSICanMessageType::I_BIN_RQ},
    {MessageType::I_BIN_PT, SSICanMessageType::I_BIN_PT},
    {MessageType::I_BIN_DT, SSICanMessageType::I_BIN_DT},
    {MessageType::I_BIN_NX, SSICanMessageType::I_BIN_NX},
    {MessageType::I_BIN_RP, SSICanMessageType::I_BIN_RP},
    {MessageType::I_BIN_AB, SSICanMessageType::I_BIN_AB},
    {MessageType::I_BIN_DN, SSICanMessageType::I_BIN_DN},
    {MessageType::I_BIN_ER, SSICanMessageType::I_BIN_ER}
};