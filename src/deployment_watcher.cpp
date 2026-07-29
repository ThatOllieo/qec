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
    if (request_) {
        gpiod_line_request_release(request_);
        request_ = nullptr;
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
    const char *chip_path = "/dev/gpiochip0";
    const unsigned int line_num = 27; // BCM 27 (deployment switch input)

    chip_ = gpiod_chip_open(chip_path);
    if (!chip_) {
        perror("gpiod_chip_open");
        running_ = false;
        return;
    }

    gpiod_line_settings* settings = gpiod_line_settings_new();
    if (!settings) {
        perror("gpiod_line_settings_new");
        running_ = false;
        return;
    }

    if (gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_INPUT) < 0) {
        perror("gpiod_line_settings_set_direction");
        gpiod_line_settings_free(settings);
        running_ = false;
        return;
    }

    if (gpiod_line_settings_set_bias(settings, GPIOD_LINE_BIAS_PULL_UP) < 0) {
        perror("gpiod_line_settings_set_bias");
        gpiod_line_settings_free(settings);
        running_ = false;
        return;
    }

    gpiod_line_config* line_config = gpiod_line_config_new();
    if (!line_config) {
        perror("gpiod_line_config_new");
        gpiod_line_settings_free(settings);
        running_ = false;
        return;
    }

    if (gpiod_line_config_add_line_settings(line_config, &line_num, 1, settings) < 0) {
        perror("gpiod_line_config_add_line_settings");
        gpiod_line_config_free(line_config);
        gpiod_line_settings_free(settings);
        running_ = false;
        return;
    }

    gpiod_request_config* request_config = gpiod_request_config_new();
    if (!request_config) {
        perror("gpiod_request_config_new");
        gpiod_line_config_free(line_config);
        gpiod_line_settings_free(settings);
        running_ = false;
        return;
    }

    gpiod_request_config_set_consumer(request_config, "gpioTest");

    request_ = gpiod_chip_request_lines(chip_, request_config, line_config);

    gpiod_request_config_free(request_config);
    gpiod_line_config_free(line_config);
    gpiod_line_settings_free(settings);

    if (!request_) {
        perror("gpiod_chip_request_lines");
        running_ = false;
        return;
    }

    // Poll loop
    while (running_) {
        int v = gpiod_line_request_get_value(request_, line_num);
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

    if (request_) {
        gpiod_line_request_release(request_);
        request_ = nullptr;
    }
    if (chip_) {
        gpiod_chip_close(chip_);
        chip_ = nullptr;
    }
}