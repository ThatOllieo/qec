#include "../include/imu.hpp"

#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>

#include <fcntl.h>      // for open()
#include <unistd.h>     // for close(), read(), write(), usleep()
#include <linux/i2c-dev.h> // I2C interface definitions
#include <sys/ioctl.h>  // for ioctl()
#include <cstdint>      // for uint8_t, int16_t
#include <iomanip>      // for output formatting
#include <deque>
#include <mutex>
#include <sstream>
#include <ctime>

//I2C path and sensor address
#define I2C_BUS "/dev/i2c-1"
#define BNO055_ADDR 0x28

//register addresses and constants
#define REG_OPR_MODE 0x3D
#define OPERATION_MODE_NDOF 0x0C
#define REG_EULER_H_LSB  0x1A
#define REG_QUATERNION_LSB 0x20
#define REG_GRAVITY_LSB 0x2E
#define REG_LIN_ACCEL_LSB 0x28
#define REG_CALIB_STAT 0x35

bool IMU::writeByte(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    return write(fd_, buf, 2) == 2;
}

bool IMU::readBytes(uint8_t reg, uint8_t* buffer, size_t length) {
    if (write(fd_, &reg, 1) != 1) return false;  // set register pointer
    return read(fd_, buffer, length) == (ssize_t)length;  // read data
}

IMU::IMU(TSQueue<Event>& mainQueue) : q_(mainQueue){}
IMU::~IMU(){
    stop();
}

void IMU::start() {
    if (running_) return;
    running_ = true;
    th_ = std::thread([this]{ loop(); });
}

void IMU::stop() {
    if (!running_) return;
    running_ = false;
    if (th_.joinable()) th_.join();
    if (fd_ >= 0) {
        close(fd_);
        fd_ = -1;
    }
    if (file.is_open()) file.close();
    // Clean up GPIO resources
}


//reads calibration status for each sensor
IMU::CalibrationStatus IMU::getCalibrationStatus() {
    uint8_t cal;
    if (!readBytes(REG_CALIB_STAT, &cal, 1)) {
        return {0, 0, 0, 0};
    }

    // Uncomment for debugging:
    // std::cout << "Raw cal byte = 0x" << std::hex << (int)cal << std::dec << std::endl;

    return {
        static_cast<uint8_t>((cal >> 6) & 0x03),
        static_cast<uint8_t>((cal >> 4) & 0x03),
        static_cast<uint8_t>((cal >> 2) & 0x03),
        static_cast<uint8_t>(cal & 0x03)
    };
}

//reads euler angles
IMU::EulerAngles IMU::getEulerAngles() {
    uint8_t buffer[6];
    if (!readBytes(REG_EULER_H_LSB, buffer, 6))
        return {0, 0, 0};

    int16_t h = (buffer[1] << 8) | buffer[0];
    int16_t r = (buffer[3] << 8) | buffer[2];
    int16_t p = (buffer[5] << 8) | buffer[4];

    return { h / 16.0, r / 16.0, p / 16.0 };
}

//quaternions
IMU::Quaternion IMU::getQuaternion() {
    uint8_t buffer[8];
    if (!readBytes(REG_QUATERNION_LSB, buffer, 8))
        return {0, 0, 0, 0};

    int16_t w = (buffer[1] << 8) | buffer[0];
    int16_t x = (buffer[3] << 8) | buffer[2];
    int16_t y = (buffer[5] << 8) | buffer[4];
    int16_t z = (buffer[7] << 8) | buffer[6];

    return { w / 16384.0, x / 16384.0, y / 16384.0, z / 16384.0 };
}

//yeah, gravity vector
IMU::Vector3 IMU::getGravity() {
    uint8_t buffer[6];
    if (!readBytes(REG_GRAVITY_LSB, buffer, 6))
        return {0, 0, 0};

    int16_t x = (buffer[1] << 8) | buffer[0];
    int16_t y = (buffer[3] << 8) | buffer[2];
    int16_t z = (buffer[5] << 8) | buffer[4];

    return { x / 100.0, y / 100.0, z / 100.0 };
}

//really doesnt need commenting after youve seen all the others
IMU::Vector3 IMU::getLinearAccel() {
    uint8_t buffer[6];
    if (!readBytes(REG_LIN_ACCEL_LSB, buffer, 6))
        return {0, 0, 0};

    int16_t x = (buffer[1] << 8) | buffer[0];
    int16_t y = (buffer[3] << 8) | buffer[2];
    int16_t z = (buffer[5] << 8) | buffer[4];

    return { x / 100.0, y / 100.0, z / 100.0 };
}

void IMU::triggerCapture(int milliseconds){
    // Capture = last buffer_ms_ of history + the next `milliseconds` into the future.
    // We don't write immediately; we arm a window and dump once the window has completed.

    if (milliseconds < 0) milliseconds = 0;

    const auto now = std::chrono::steady_clock::now();

    std::lock_guard<std::mutex> lk(mtx_);

    // If a capture is already active, ignore (or you could restart it).
    if (capture_active_) return;

    capture_active_ = true;
    capture_future_ms_ = milliseconds;

    capture_ref_   = now; // reference (t=0) for the capture
    capture_start_ = capture_ref_ - std::chrono::milliseconds(buffer_ms_);
    capture_end_   = capture_ref_ + std::chrono::milliseconds(milliseconds);

    // Ensure buffer doesn't hold more than needed.
    while (!buffer_.empty() && buffer_.front().t < capture_start_) {
        buffer_.pop_front();
    }
}

void IMU::dumpCaptureToCsvLocked(const std::string& path,
                                const std::chrono::steady_clock::time_point& start,
                                const std::chrono::steady_clock::time_point& end,
                                const std::chrono::steady_clock::time_point& ref)
{
    // mtx_ must already be held by caller.

    std::ofstream out(path);
    if (!out.is_open()) {
        std::cerr << "Failed to open capture file: " << path << "\n";
        return;
    }

    // CSV header
    out << "t_ms,"
        << "linAx,linAy,linAz,"
        << "gravX,gravY,gravZ,"
        << "heading,roll,pitch,"
        << "quatW,quatX,quatY,quatZ,"
        << "calSys,calGyro,calAccel,calMag\n";

    for (const auto& s : buffer_) {
        if (s.t < start) continue;
        if (s.t > end) break;

        // dt is relative to trigger time (ref). Pre-trigger samples will be negative.
        const auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(s.t - ref).count();

        out << dt << ','
            << s.linAcc.x << ',' << s.linAcc.y << ',' << s.linAcc.z << ','
            << s.grav.x   << ',' << s.grav.y   << ',' << s.grav.z   << ','
            << s.eul.heading << ',' << s.eul.roll << ',' << s.eul.pitch << ','
            << s.quat.w << ',' << s.quat.x << ',' << s.quat.y << ',' << s.quat.z << ','
            << static_cast<int>(s.cal.sys) << ','
            << static_cast<int>(s.cal.gyro) << ','
            << static_cast<int>(s.cal.accel) << ','
            << static_cast<int>(s.cal.mag)
            << '\n';
    }

    out.close();
}

void IMU::loop(){
    // Open I2C device file
    fd_ = open(I2C_BUS, O_RDWR);
    if (fd_ < 0) {
        perror("Failed to open I2C bus");
        return;
    }

    // Set the I2C address for the BNO055
    if (ioctl(fd_, I2C_SLAVE, BNO055_ADDR) < 0) {
        perror("Failed to connect to BNO055");
        close(fd_);
        return;
    }

    // Set the BNO055 to NDOF (fusion) mode
    if (!writeByte(REG_OPR_MODE, OPERATION_MODE_NDOF)) {
        std::cerr << "Failed to set fusion mode\n";
        close(fd_);
        return;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(20)); // Wait 20 ms after mode change

    while(running_){
        auto start_time = std::chrono::steady_clock::now();

        Vector3 linAcc = getLinearAccel();
        Vector3 grav = getGravity();
        EulerAngles eul = getEulerAngles();
        Quaternion quat = getQuaternion();
        CalibrationStatus cal = getCalibrationStatus();

        // Build one sample
        Sample s;
        s.t = start_time;
        s.linAcc = linAcc;
        s.grav = grav;
        s.eul = eul;
        s.quat = quat;
        s.cal = cal;

        bool should_dump = false;
        std::chrono::steady_clock::time_point dump_start;
        std::chrono::steady_clock::time_point dump_end;
        std::chrono::steady_clock::time_point dump_ref;
        std::string dump_path;

        {
            std::lock_guard<std::mutex> lk(mtx_);

            buffer_.push_back(s);

            // Keep only the last buffer_ms_ of data normally.
            // But if a capture is armed, we must retain data back to capture_start_
            // (otherwise by the time we reach capture_end_ we'd have dropped the pre-trigger samples).
            auto keep_from = start_time - std::chrono::milliseconds(buffer_ms_);
            if (capture_active_) {
                // capture_start_ is earlier than (start_time - buffer_ms_) once time has advanced,
                // so keep_from must be the earlier of the two.
                if (capture_start_ < keep_from) keep_from = capture_start_;
            }
            while (!buffer_.empty() && buffer_.front().t < keep_from) {
                buffer_.pop_front();
            }

            // If capture is active and we've collected past the capture end, dump once.
            if (capture_active_ && start_time >= capture_end_) {
                dump_start = capture_start_;
                dump_end   = capture_end_;
                dump_ref   = capture_ref_;

                // Simple filename: imu_capture_<index>.csv
                std::ostringstream oss;
                oss << "/home/pi/imuCaptures/imu_capture_" << capture_index_++ << ".csv";
                dump_path = oss.str();

                // Mark capture as completed before writing so triggerCapture can be used again.
                capture_active_ = false;
                should_dump = true;
            }
        }

        if (should_dump) {
            std::lock_guard<std::mutex> lk(mtx_);
            dumpCaptureToCsvLocked(dump_path, dump_start, dump_end, dump_ref);
            std::cout << "Wrote IMU capture to " << dump_path << "\n";

            Event ev{
                EventType::MotionCaptured,
                EvMotionCaptured{dump_path},
                0
            };
            q_.push(std::move(ev));

        }

        // Rate limit: aim for ~sample_period_ms_
        const auto elapsed = std::chrono::steady_clock::now() - start_time;
        const auto target = std::chrono::milliseconds(sample_period_ms_);
        if (elapsed < target) {
            std::this_thread::sleep_for(target - elapsed);
        }
    }
}