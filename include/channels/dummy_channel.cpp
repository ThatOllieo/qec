#include "dummy_channel.hpp"
#include <iostream>

DummyChannel::DummyChannel() {}
DummyChannel::~DummyChannel() { stop(); }

bool DummyChannel::start() {
    running_ = true;
    std::cout << "[DummyChannel] started\n";
    return true;
}

void DummyChannel::stop() {
    if (!running_) return;
    running_ = false;
    std::cout << "[DummyChannel] stopped\n";
}

void DummyChannel::send(const CommsMessage& msg) {
    std::cout << "[DummyChannel] send called: type=0x"
              << std::hex << static_cast<uint16_t>(msg.type)
              << " cid=" << std::dec << msg.correlation_id << "\n";
}

void DummyChannel::inject(const CommsMessage& msg) {
    if (on_receive_) {
        std::cout << "[DummyChannel] inject (simulating RX)\n";
        on_receive_(msg);
    } else {
        std::cout << "[DummyChannel] inject called but no on_receive registered\n";
    }
}