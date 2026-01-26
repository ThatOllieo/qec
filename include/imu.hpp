#pragma once
#include <atomic>
#include <thread>
#include <cstdint>
#include "../include/tsqueue.hpp"
#include "../include/events.hpp"
#include <fstream>
#include <deque>
#include <mutex>
#include <chrono>
#include <string>

class IMU {
public:
    explicit IMU(TSQueue<Event>& mainQueue);
    ~IMU();

    void start();
    void stop();

    struct EulerAngles {
        double heading;
        double roll;
        double pitch;
    };

    struct Quaternion {
        double w, x, y, z;
    };

    struct Vector3 {
        double x, y, z;
    };

    struct CalibrationStatus{
        uint8_t sys;
        uint8_t gyro;
        uint8_t accel;
        uint8_t mag;
        bool fullyCalibrated() const {
            // All 4 must be max value (3)
            return sys == 3 && gyro == 3 && accel == 3 && mag == 3;
        }
    };

    struct Sample {
        std::chrono::steady_clock::time_point t;
        Vector3 linAcc;
        Vector3 grav;
        EulerAngles eul;
        Quaternion quat;
        CalibrationStatus cal;
    };

    CalibrationStatus getCalibrationStatus();
    EulerAngles getEulerAngles();
    Quaternion getQuaternion();
    Vector3 getGravity();
    Vector3 getLinearAccel();

    void triggerCapture(int milliseconds);

private:
    void loop();

    TSQueue<Event>& q_;
    std::thread th_;
    std::atomic<bool> running_{false};

    int fd_{-1};
    std::ofstream file;

    bool writeByte(uint8_t reg, uint8_t value);
    bool readBytes(uint8_t reg, uint8_t* buffer, size_t length);

    // Ring buffer length (how much history we keep in RAM)
    int buffer_ms_ = 10000; // 10 seconds

    // IMU sampling period (approx). 20 ms = 50 Hz.
    int sample_period_ms_ = 20;

    // Buffer of recent samples
    std::deque<Sample> buffer_;
    std::mutex mtx_;

    // Capture state
    bool capture_active_ = false;
    int capture_future_ms_ = 0;
    std::chrono::steady_clock::time_point capture_start_;
    std::chrono::steady_clock::time_point capture_end_;
    std::chrono::steady_clock::time_point capture_ref_{}; // time triggerCapture() was called (t=0)

    // Simple capture file naming
    uint64_t capture_index_ = 0;

    void dumpCaptureToCsvLocked(const std::string& path,
                               const std::chrono::steady_clock::time_point& start,
                               const std::chrono::steady_clock::time_point& end,
                               const std::chrono::steady_clock::time_point& ref);
};