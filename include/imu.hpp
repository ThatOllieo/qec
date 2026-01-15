#pragma once
#include <atomic>
#include <thread>
#include <cstdint>
#include "../include/tsqueue.hpp"
#include "../include/events.hpp"
#include <fstream>

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


    int a = 0;
    int t = 10000;

};