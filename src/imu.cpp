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

// Define I2C path and sensor address
#define I2C_BUS "/dev/i2c-1"
#define BNO055_ADDR 0x28

// Define register addresses and constants
#define REG_OPR_MODE 0x3D
#define OPERATION_MODE_NDOF 0x0C
#define REG_EULER_H_LSB  0x1A
#define REG_QUATERNION_LSB 0x20
#define REG_GRAVITY_LSB 0x2E
#define REG_LIN_ACCEL_LSB 0x28
#define REG_CALIB_STAT 0x35


// Writes a single byte value to a register on the BNO055
bool IMU::writeByte(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    return write(fd_, buf, 2) == 2;
}

// Reads multiple bytes from a register
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
    if (file.is_open()) file.close();
    // Clean up GPIO resources
}

//reads calibration status for each sensor
IMU::CalibrationStatus IMU::getCalibrationStatus() {
    uint8_t cal;
    if (!readBytes(REG_CALIB_STAT, &cal, 1)) {
        return {0, 0, 0, 0};
    }

    std::cout << "Raw cal byte = 0x" << std::hex << (int)cal << std::dec << std::endl;

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
    t = milliseconds;
    a = 1;
}

void IMU::loop() {
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

    //while (running_) {
    while(running_){
        if(a == 1){

            file.open("/home/pi/startup.csv", std::ios::out | std::ios::trunc);
            if (!file.is_open()) {
                std::perror("Failed to open output file");
                return;
            }
            auto start_time = std::chrono::steady_clock::now();
            file << "time_ms,ax,ay,az,h,r,p\n";
            file.setf(std::ios::fixed);
            file << std::setprecision(4);

            while(a == 1){
                auto now = std::chrono::steady_clock::now();
                auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();
                if(elapsed <= t){
                    Vector3 dat = getLinearAccel();
                    EulerAngles dat2 = getEulerAngles();
                    file << elapsed << "," << dat.x << "," << dat.y << "," << dat.z << "," << dat2.heading << "," << dat2.roll << "," << dat2.pitch << "\n";
                    file.flush();
                }
                else{
                    a = 0;
                    if (file.is_open()) file.close();
                    Event ev{
                        EventType::MotionCaptured,
                        EvMotionCaptured{std::string("/home/pi/startup.csv")},
                        0
                    };
                    q_.push(std::move(ev));

                    //system("scp /home/pi/startup.csv pi@10.42.0.1:/home/pi/startup.csv");
                    
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }


        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (file.is_open()) file.flush();
    if (fd_ >= 0) close(fd_);
    return;
}
