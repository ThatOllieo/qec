#pragma once

#include <atomic>
#include <thread>
#include <string>
#include <memory>

#include <boost/asio.hpp>
#include <boost/beast.hpp>
#include <boost/beast/websocket.hpp>

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