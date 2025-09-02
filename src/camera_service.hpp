#pragma once
#include <cstdint>
#include <string>

// Your existing headers:
#include "../include/events.hpp"   // Event, EventType, EvPhotoTaken
#include "../include/tsqueue.hpp"  // TSQueue<Event>

struct CameraModuleConfig {
    int      left_index   = 0;     // physical camera index for "left" (0-based)
    int      right_index  = 1;     // physical camera index for "right"
    unsigned width        = 1920;
    unsigned height       = 1080;
    int      warmup_frames = 8;    // run before every capture
    int      jpeg_quality  = 85;   // 1..100
};

class CameraModule {
public:
    // Construct with a reference to your main event queue; module will push PhotoTaken events into it.
    explicit CameraModule(TSQueue<Event>& mainEventQueue);
    ~CameraModule();

    // One-shot startup that configures BOTH cameras (by index) and launches internal threads.
    // Returns true if both requested cameras were started successfully.
    bool startup(const CameraModuleConfig& cfg);

    // Non-blocking capture requests. Each will warm-up, capture, JPEG, then push a PhotoTaken event.
    void take_left (const std::string& path, uint32_t seq = 0);
    void take_right(const std::string& path, uint32_t seq = 0);
    // Convenience: trigger both near-simultaneously (software-synced)
    void take_both (const std::string& left_path, const std::string& right_path, uint32_t seq = 0);

    // Graceful shutdown of both cameras/threads.
    void shutdown();

    CameraModule(const CameraModule&) = delete;
    CameraModule& operator=(const CameraModule&) = delete;

private:
    struct Impl;
    Impl* d_;  // pImpl to keep libcamera out of main
};
