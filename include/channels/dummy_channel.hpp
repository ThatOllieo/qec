#pragma once
#include "ichannel.hpp"

class DummyChannel : public IChannel {
public:
    DummyChannel();
    ~DummyChannel() override;

    bool start() override;
    void stop() override;
    void send(const CommsMessage& msg) override;

    // helper so tests can simulate an inbound frame
    void inject(const CommsMessage& msg);

private:
    bool running_ = false;
};