#pragma once
#include <atomic>
#include <thread>
#include <gpiod.h>
#include "../include/tsqueue.hpp"
#include "../include/events.hpp"

class DeploymentWatcher {
public:
    explicit DeploymentWatcher(TSQueue<Event>& mainQueue);
    ~DeploymentWatcher();

    void start();
    void stop();

    bool getState();

private:
    void loop();

    TSQueue<Event>& q_;
    std::thread th_;
    std::atomic<bool> running_{false};
    std::atomic<bool> deployed_{false};
    gpiod_chip* chip_ = nullptr;
    gpiod_line* line_ = nullptr;   
};