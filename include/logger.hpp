#pragma once
#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <string>

#include "exceptions.hpp"

// Simple, crash-proof file logger. Mirrors everything to stderr as well so it is
// a strict superset of the previous std::cerr-only behaviour. Never throws -
// if the log file can't be opened/written, it silently degrades to stderr-only.
class Logger {
public:
    static Logger& instance() {
        static Logger inst;
        return inst;
    }

    void log(ErrorSeverity sev, const std::string& tag, const std::string& msg) {
        std::lock_guard<std::mutex> lk(mx_);

        std::string line = timestamp() + " " + severityTag(sev) + " [" + tag + "] " + msg;

        std::cerr << line << "\n";

        if (file_ok_) {
            try {
                file_ << line << "\n";
                file_.flush();
                if (file_.fail()) file_ok_ = false;
            } catch (...) {
                file_ok_ = false;
            }
        }
    }

private:
    Logger() {
        try {
            std::filesystem::create_directories("/home/pi/logs");
            file_.open("/home/pi/logs/uk.log", std::ios::app);
            file_ok_ = file_.is_open();
        } catch (...) {
            file_ok_ = false;
        }
    }

    static std::string timestamp() {
        auto now = std::chrono::system_clock::now();
        std::time_t t = std::chrono::system_clock::to_time_t(now);
        std::tm tm_buf{};
        localtime_r(&t, &tm_buf);
        char buf[32];
        std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &tm_buf);
        return std::string(buf);
    }

    std::mutex mx_;
    std::ofstream file_;
    bool file_ok_ = false;
};

inline void logLine(ErrorSeverity sev, const std::string& tag, const std::string& msg) {
    Logger::instance().log(sev, tag, msg);
}
