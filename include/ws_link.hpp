#pragma once

#include "../include/tsqueue.hpp"
#include "../include/events.hpp"

class WSLink {
public:
    explicit WSLink(TSQueue<Event>& mainQueue);
    ~WSLink();

    void start(uint16_t port = 9002);
    void stop();

    void broadcast(std::string jsonText);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};