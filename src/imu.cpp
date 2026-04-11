#include "../include/imu.hpp"

#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>
#include <stdexcept>

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
    std::lock_guard<std::mutex> lk(device_mtx_);
    if (fd_ < 0) return false;
    uint8_t buf[2] = {reg, value};
    return write(fd_, buf, 2) == 2;
}

bool IMU::readBytes(uint8_t reg, uint8_t* buffer, size_t length) {
    std::lock_guard<std::mutex> lk(device_mtx_);
    if (fd_ < 0) return false;
    if (write(fd_, &reg, 1) != 1) return false;
    return read(fd_, buffer, length) == static_cast<ssize_t>(length);
}

IMU::IMU(TSQueue<Event>& mainQueue) : q_(mainQueue){}
IMU::~IMU(){
    stop();
}

void IMU::cleanupHardware() noexcept {
    {
        std::lock_guard<std::mutex> lk(device_mtx_);
        if (fd_ >= 0) {
            close(fd_);
            fd_ = -1;
        }
    }

    if (file.is_open()) file.close();
}

void IMU::setFailedState(const std::string& message) {
    {
        std::lock_guard<std::mutex> lk(error_mtx_);
        last_error_ = message;
    }
    state_ = ModuleState::Failed;
    running_ = false;
}

std::string IMU::getLastError() const {
    std::lock_guard<std::mutex> lk(error_mtx_);
    return last_error_;
}

void IMU::ensureSampleAvailable(const char* caller) const {
    const auto state = state_.load();
    if (state == ModuleState::Failed) {
        throw IMUError(
            ErrorCode::StateError,
            ErrorSeverity::Recoverable,
            "Attempted to read cached IMU data while module is failed",
            caller,
            {},
            getLastError()
        );
    }

    if (!latest_sample_valid_) {
        throw IMUError(
            ErrorCode::StateError,
            ErrorSeverity::Recoverable,
            "Attempted to read IMU data before a valid sample was available",
            caller
        );
    }
}

void IMU::start() {
    if (running_) return;

    state_ = ModuleState::Starting;
    {
        std::lock_guard<std::mutex> lk(error_mtx_);
        last_error_.clear();
    }

    try {
        startupTasks();

        running_ = true;
        th_ = std::thread([this] {
            try {
                loop();
            } catch (const IMUError& ex) {
                if(ex.severity == ErrorSeverity::Fatal){std::cerr << "[FATAL]"}
                else if(ex.severity == ErrorSeverity::Recoverable){std::cerr << "[ERROR]";}
                else{std::cerr << "[WARNING]";}
                std::cerr << "[IMU] Worker thread failed: " << ex.what() << "\n";
                setFailedState(ex.what());
                cleanupHardware();
            } catch (const std::exception& ex) {
                std::cerr << "[?][IMU] Worker thread failed with std::exception: " << ex.what() << "\n";
                setFailedState(ex.what());
                cleanupHardware();
            } catch (...) {
                std::cerr << "[?][IMU] Worker thread failed with unknown exception\n";
                setFailedState("Unknown IMU worker failure");
                cleanupHardware();
            }
        });
    } catch (...) {
        running_ = false;
        cleanupHardware();
        state_ = ModuleState::Failed;
        throw;
    }
}

void IMU::stop() {
    running_ = false;

    if (th_.joinable()) {
        if (th_.get_id() == std::this_thread::get_id()) {
            th_.detach();
        } else {
            th_.join();
        }
    }

    cleanupHardware();

    if (state_.load() != ModuleState::Failed) {
        state_ = ModuleState::Stopped;
    }
}

IMU::CalibrationStatus IMU::getCalibrationStatus() const {
    std::lock_guard<std::mutex> lk(latest_sample_mtx_);
    ensureSampleAvailable("IMU::getCalibrationStatus");
    return latest_sample_.cal;
}

IMU::EulerAngles IMU::getEulerAngles() const {
    std::lock_guard<std::mutex> lk(latest_sample_mtx_);
    ensureSampleAvailable("IMU::getEulerAngles");
    return latest_sample_.eul;
}

IMU::Quaternion IMU::getQuaternion() const {
    std::lock_guard<std::mutex> lk(latest_sample_mtx_);
    ensureSampleAvailable("IMU::getQuaternion");
    return latest_sample_.quat;
}

IMU::Vector3 IMU::getGravity() const {
    std::lock_guard<std::mutex> lk(latest_sample_mtx_);
    ensureSampleAvailable("IMU::getGravity");
    return latest_sample_.grav;
}

IMU::Vector3 IMU::getLinearAccel() const {
    std::lock_guard<std::mutex> lk(latest_sample_mtx_);
    ensureSampleAvailable("IMU::getLinearAccel");
    return latest_sample_.linAcc;
}

IMU::CalibrationStatus IMU::readCalibrationStatusHw() {
    uint8_t cal = 0;
    if (!readBytes(REG_CALIB_STAT, &cal, 1)) {
        throw IMUError(
            ErrorCode::IOError,
            ErrorSeverity::Recoverable,
            "Failed to read calibration status",
            "IMU::readCalibrationStatusHw"
        );
    }

    return {
        static_cast<uint8_t>((cal >> 6) & 0x03),
        static_cast<uint8_t>((cal >> 4) & 0x03),
        static_cast<uint8_t>((cal >> 2) & 0x03),
        static_cast<uint8_t>(cal & 0x03)
    };
}

IMU::EulerAngles IMU::readEulerAnglesHw() {
    uint8_t buffer[6]{};
    if (!readBytes(REG_EULER_H_LSB, buffer, 6)) {
        throw IMUError(
            ErrorCode::IOError,
            ErrorSeverity::Recoverable,
            "Failed to read Euler angles",
            "IMU::readEulerAnglesHw"
        );
    }

    int16_t h = static_cast<int16_t>((buffer[1] << 8) | buffer[0]);
    int16_t r = static_cast<int16_t>((buffer[3] << 8) | buffer[2]);
    int16_t p = static_cast<int16_t>((buffer[5] << 8) | buffer[4]);

    return { h / 16.0, r / 16.0, p / 16.0 };
}

IMU::Quaternion IMU::readQuaternionHw() {
    uint8_t buffer[8]{};
    if (!readBytes(REG_QUATERNION_LSB, buffer, 8)) {
        throw IMUError(
            ErrorCode::IOError,
            ErrorSeverity::Recoverable,
            "Failed to read quaternion",
            "IMU::readQuaternionHw"
        );
    }

    int16_t w = static_cast<int16_t>((buffer[1] << 8) | buffer[0]);
    int16_t x = static_cast<int16_t>((buffer[3] << 8) | buffer[2]);
    int16_t y = static_cast<int16_t>((buffer[5] << 8) | buffer[4]);
    int16_t z = static_cast<int16_t>((buffer[7] << 8) | buffer[6]);

    return { w / 16384.0, x / 16384.0, y / 16384.0, z / 16384.0 };
}

IMU::Vector3 IMU::readGravityHw() {
    uint8_t buffer[6]{};
    if (!readBytes(REG_GRAVITY_LSB, buffer, 6)) {
        throw IMUError(
            ErrorCode::IOError,
            ErrorSeverity::Recoverable,
            "Failed to read gravity vector",
            "IMU::readGravityHw"
        );
    }

    int16_t x = static_cast<int16_t>((buffer[1] << 8) | buffer[0]);
    int16_t y = static_cast<int16_t>((buffer[3] << 8) | buffer[2]);
    int16_t z = static_cast<int16_t>((buffer[5] << 8) | buffer[4]);

    return { x / 100.0, y / 100.0, z / 100.0 };
}

IMU::Vector3 IMU::readLinearAccelHw() {
    uint8_t buffer[6]{};
    if (!readBytes(REG_LIN_ACCEL_LSB, buffer, 6)) {
        throw IMUError(
            ErrorCode::IOError,
            ErrorSeverity::Recoverable,
            "Failed to read linear acceleration",
            "IMU::readLinearAccelHw"
        );
    }

    int16_t x = static_cast<int16_t>((buffer[1] << 8) | buffer[0]);
    int16_t y = static_cast<int16_t>((buffer[3] << 8) | buffer[2]);
    int16_t z = static_cast<int16_t>((buffer[5] << 8) | buffer[4]);

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

void IMU::startupTasks() {
    fd_ = open(I2C_BUS, O_RDWR);
    if (fd_ < 0) {
        throw IMUError(
            ErrorCode::IOError,
            ErrorSeverity::Recoverable,
            "Failed to open I2C bus",
            "IMU::startupTasks"
        );
    }

    try {
        {
            std::lock_guard<std::mutex> lk(device_mtx_);
            if (ioctl(fd_, I2C_SLAVE, BNO055_ADDR) < 0) {
                throw IMUError(
                    ErrorCode::DeviceUnreachable,
                    ErrorSeverity::Recoverable,
                    "Failed to connect to BNO055",
                    "IMU::startupTasks"
                );
            }
        }

        if (!writeByte(REG_OPR_MODE, OPERATION_MODE_NDOF)) {
            throw IMUError(
                ErrorCode::IOError,
                ErrorSeverity::Recoverable,
                "Failed to set fusion mode",
                "IMU::startupTasks"
            );
        }
    } catch (...) {
        cleanupHardware();
        throw;
    }
}

void IMU::loop() {
    state_ = ModuleState::Running;

    while (running_) {
        const auto start_time = std::chrono::steady_clock::now();

        Sample s;
        s.t = start_time;
        s.linAcc = readLinearAccelHw();
        s.grav   = readGravityHw();
        s.eul    = readEulerAnglesHw();
        s.quat   = readQuaternionHw();
        s.cal    = readCalibrationStatusHw();

        {
            std::lock_guard<std::mutex> latest_lk(latest_sample_mtx_);
            latest_sample_ = s;
            latest_sample_valid_ = true;
        }

        bool should_dump = false;
        std::chrono::steady_clock::time_point dump_start;
        std::chrono::steady_clock::time_point dump_end;
        std::chrono::steady_clock::time_point dump_ref;
        std::string dump_path;

        {
            std::lock_guard<std::mutex> lk(mtx_);

            buffer_.push_back(s);

            auto keep_from = start_time - std::chrono::milliseconds(buffer_ms_);
            if (capture_active_ && capture_start_ < keep_from) {
                keep_from = capture_start_;
            }
            while (!buffer_.empty() && buffer_.front().t < keep_from) {
                buffer_.pop_front();
            }

            if (capture_active_ && start_time >= capture_end_) {
                dump_start = capture_start_;
                dump_end   = capture_end_;
                dump_ref   = capture_ref_;

                std::ostringstream oss;
                oss << "/home/pi/imuCaptures/imu_capture_" << capture_index_++ << ".csv";
                dump_path = oss.str();

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

        const auto elapsed = std::chrono::steady_clock::now() - start_time;
        const auto target = std::chrono::milliseconds(sample_period_ms_);
        if (elapsed < target) {
            std::this_thread::sleep_for(target - elapsed);
        }
    }
}