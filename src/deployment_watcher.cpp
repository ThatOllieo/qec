#include "../include/deployment_watcher.hpp"

#include <gpiod.h>
#include <iostream>
#include <chrono>
#include <thread>

DeploymentWatcher::DeploymentWatcher(TSQueue<Event>& mainQueue) : q_(mainQueue) {}

DeploymentWatcher::~DeploymentWatcher() {
    stop();
}

void DeploymentWatcher::start() {
    if (running_) return;
    running_ = true;
    th_ = std::thread([this]{ loop(); });
}

void DeploymentWatcher::stop() {
    if (!running_) return;
    running_ = false;
    if (th_.joinable()) th_.join();

    // Clean up GPIO resources
    if (line_) {
        gpiod_line_release(line_);
        line_ = nullptr;
    }
    if (chip_) {
        gpiod_chip_close(chip_);
        chip_ = nullptr;
    }
}

bool DeploymentWatcher::getState() {
    return deployed_.load();
}

void DeploymentWatcher::loop() {
    // Open once when the thread starts
    const char *chipname = "gpiochip0";
    unsigned int line_num = 27; // BCM 17 (Inspire live version has deployment switch connections on both 27 and 17, 27 is the side one)

    chip_ = gpiod_chip_open_by_name(chipname);
    if (!chip_) {
        perror("gpiod_chip_open_by_name");
        running_ = false;
        return;
    }

    line_ = gpiod_chip_get_line(chip_, line_num);
    if (!line_) {
        perror("gpiod_chip_get_line");
        running_ = false;
        return;
    }

    gpiod_line_request_config config = {
        "gpioTest",
        GPIOD_LINE_REQUEST_DIRECTION_INPUT,
        GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP
    };

    if (gpiod_line_request(line_, &config, 0) < 0) {
        perror("gpiod_line_request");
        running_ = false;
        return;
    }

    // Poll loop
    while (running_) {
        int v = gpiod_line_get_value(line_);
        if (v < 0) {
            perror("gpiod_line_get_value");
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

        if (v == 1) {
            if (!deployed_) {
                Event ev{
                    EventType::DeploymentTriggered,
                    EvDeploymentTriggered{'d'},
                    0
                };
                q_.push(std::move(ev));
                deployed_ = true;
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        } else {
            //std::cout << "DPLY RST" << std::endl;
            deployed_ = false;
        }

        // Small sleep to avoid busy loop (and basic debounce)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (line_) {
        gpiod_line_release(line_);
        line_ = nullptr;
    }
    if (chip_) {
        gpiod_chip_close(chip_);
        chip_ = nullptr;
    }
}