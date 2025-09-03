#pragma once
#include <cstdint>
#include <string>
#include <variant>
#include <vector>
#include "comms_message.hpp"

//============ EVENT TYPES ==========
enum class EventType {
    PhotoTaken,
    MotionCaptured,         
    NeedTelem,          
    DeploymentTriggered, 
    Command,
    TelemetryRequest,
    TelemetryArrived,
    TelemetryFailed,
    CommandAcked,
    CommandFailed,
};

//=========== PAYLOAD TYPES ===========
struct EvPhotoTaken {
    std::string path;      // e.g., "/data/imgs/pic_001.jpg"
};

struct EvMotionCaptured {
    std::string path;
};

struct EvDeploymentTriggered {
    char key;              // which key was pressed, for debug (e.g., 'd') depreciated, just fill with d for deploy
};

struct EvCommand {
    uint16_t correlation_id;
    uint16_t command_id;
    uint8_t requester_id;
    uint8_t reply_via;
    std::vector<uint8_t> args;
};

struct EvTelemetryRequest {
    uint16_t correlation_id;
    uint16_t sensor_id;
    uint8_t requester_id;
    uint8_t reply_via;
    std::vector<uint8_t> params;
};

struct EvTelemetryArrived { uint16_t correlation_id; std::vector<uint8_t> bytes;};
struct EvTelemetryFailed { uint16_t correlation_id; std::string reason; };
struct EvCommandAcked { uint16_t correlation_id; };
struct EvCommandFailed { uint16_t correlation_id; std::string reason; };

//======= WRAPPER =======
using EventPayload = std::variant<
    EvPhotoTaken, 
    EvMotionCaptured,
    EvDeploymentTriggered, 
    EvCommand, 
    EvTelemetryRequest,
    EvTelemetryArrived,
    EvTelemetryFailed,
    EvCommandAcked,
    EvCommandFailed
>;

//========= EVENT STRUCTURE =======
struct Event {
    EventType type;
    EventPayload data;
    uint64_t seq = 0;  // optional: sequence for logging
};
