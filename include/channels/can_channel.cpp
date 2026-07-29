#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include "can_channel.hpp"

#include <iostream>


void push_bits(std::vector<bool>& frame, uint32_t value, int num_bits) {
    for (int i = num_bits - 1; i >= 0; --i) {
        frame.push_back((value >> i) & 1);
    }
}

uint32_t extract_bits(const std::vector<bool>& frame, size_t offset, size_t len) {
    uint32_t value = 0;
    for (size_t i = 0; i < len; ++i) {
        value = (value << 1) | (frame[offset + i] ? 1 : 0);
    }
    return value;
}

CanChannel::CanChannel(CanConfig cfg) : cfg_(std::move(cfg)){}
CanChannel::~CanChannel() { stop(); }

bool CanChannel::start(){
    if(running_){
        return true;
    }
    set_state(ChannelState::Starting);

    if (!hw_open()) {
        set_state(ChannelState::Failed);
        return false;
    }

    running_ = true;
    set_state(ChannelState::Running);

    rx_thread_ = std::thread(&CanChannel::rx_loop, this);
    tx_thread_ = std::thread(&CanChannel::tx_loop, this);
    return true;
}

void CanChannel::stop(){
    if(!running_) return;
    running_ = false;

    tx_frames_.push(can_frame{});
    if(tx_thread_.joinable()) tx_thread_.join();
    if(rx_thread_.joinable()) rx_thread_.join(); // rx_loop wakes within SO_RCVTIMEO, sees running_==false, exits
    hw_close();
    set_state(ChannelState::Stopped);
}

bool CanChannel::send(const CommsMessage& msg){
    if (state_.load(std::memory_order_relaxed) != ChannelState::Running) {
        return false;
    }

    struct can_frame frame {};
    if (!serialize(frame, msg)) {
        return false;
    }

    tx_frames_.push(std::move(frame));
    return true;
}

void CanChannel::tx_loop() {
    try {
        while (running_) {
            auto frame = tx_frames_.pop();
            if (!running_) break;

            if (!write_frame(frame)) {
                set_state(ChannelState::Failed);
                break;
            }
        }
    } catch (const std::exception& e) {
        set_state(ChannelState::Failed);
    }
}

void CanChannel::rx_loop() {
    try {
        while (running_) {
            struct can_frame frame;
            errno = 0;
            if (!read_frame(frame)) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    continue; // SO_RCVTIMEO timeout — just recheck running_
                }
                set_state(ChannelState::Failed);
                break;
            }

            CommsMessage msg{};
            if (parse(frame, msg)) {
                if (rx_cb_) rx_cb_(msg);
            }
        }
    } catch (const std::exception& e) {
        set_state(ChannelState::Failed);
    }
}

bool CanChannel::hw_open() {
    // Implementation for opening the CAN interface
    can_fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_fd_ < 0) {
        return false;
    }

    struct ifreq ifr {};
    std::strncpy(ifr.ifr_name, cfg_.interface.c_str(), IFNAMSIZ - 1);
    if (::ioctl(can_fd_, SIOCGIFINDEX, &ifr) < 0) {
        ::close(can_fd_);
        return false;
    }

    struct sockaddr_can addr {};
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (::bind(can_fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        ::close(can_fd_);
        return false;
    }
    
    struct can_filter filt {};
    filt.can_id   = 0;
    filt.can_mask = 0;
    if (::setsockopt(can_fd_, SOL_CAN_RAW, CAN_RAW_FILTER, &filt, sizeof(filt)) < 0) {
        ::close(can_fd_);
        return false;
    }

    can_err_mask_t err_mask = CAN_ERR_MASK;
    if (::setsockopt(can_fd_, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask, sizeof(err_mask)) < 0) {
        // Not fatal, continue without error frame visibility
    }

    // Bounds how long read_frame() can block, so rx_loop() periodically
    // wakes up to recheck running_ instead of relying on stop() closing
    // the fd out from under a blocked read().
    struct timeval rx_timeout {};
    rx_timeout.tv_sec  = 0;
    rx_timeout.tv_usec = 200000; // 200ms
    if (::setsockopt(can_fd_, SOL_SOCKET, SO_RCVTIMEO, &rx_timeout, sizeof(rx_timeout)) < 0) {
        ::close(can_fd_);
        return false;
    }

    return true; // Placeholder
}

void CanChannel::hw_close() {
    // Implementation for closing the CAN interface
    ::close(can_fd_);
}

bool CanChannel::write_frame(const struct can_frame& frame) {
    // Implementation for writing a frame to the CAN interface
    ssize_t nbytes = ::write(can_fd_, &frame, sizeof(frame));
    if (nbytes != sizeof(frame)) {
        return false;
    }
    std::cerr << "Can frame written :D" << std::endl;
    return true; // Placeholder
}

bool CanChannel::read_frame(struct can_frame& frame) {
    // Implementation for reading a frame from the CAN interface.
    // errno is left set on failure so rx_loop() can tell a SO_RCVTIMEO
    // timeout (EAGAIN/EWOULDBLOCK, not a real error) apart from an
    // actual read failure.
    ssize_t nbytes = ::read(can_fd_, &frame, sizeof(frame));
    if (nbytes < 0) {
        return false;
    }
    else if (nbytes < static_cast<ssize_t>(sizeof(can_frame))) {
        return false;
    }

    return true;
}

bool CanChannel::serialize(struct can_frame& frame, const CommsMessage& msg) const {
    // Implementation for serializing a CommsMessage into a CAN frame
    //fixed frame of [1b priority][4b from][4b to][4b type][12b message][4b frame number][64b payload]
    std::vector<bool> header;
    header.reserve(1+4+4+4+12+4+64);
    push_bits(header, 0x0, 1); // priority
    if (msg.src == cfg_.sat_src) {
        push_bits(header, 0x00, 4);
    } else {
        push_bits(header, (msg.src & 0x0F), 4);
    }
    if (msg.dest == cfg_.sat_src) {
        push_bits(header, 0x00, 4);
    } else {
        push_bits(header, (msg.dest & 0x0F), 4);
    }
    push_bits(header, (static_cast<uint8_t>(MessageTypeToSSICanMessageType[msg.type]) & 0xF), 4);
    push_bits(header, msg.command_or_sensor_id & 0xFFF, 12);
    push_bits(header, 0x0, 4);
    std::vector<uint8_t> payload;
    for (size_t i = 0; i < 8 && i < msg.payload.size(); ++i) {
        payload.push_back(msg.payload[i]);
    }

    frame.can_id = 0;
    for (size_t i = 0; i < header.size(); ++i) {
        frame.can_id |= (header[i] ? 1 : 0) << (28 - i);
    }
    frame.can_id |= CAN_EFF_FLAG; // Set the extended frame format flag
    frame.can_dlc = payload.size();
    std::memcpy(frame.data, payload.data(), payload.size());

    return true;
}

bool CanChannel::parse(struct can_frame& frame, CommsMessage& msg) const {
    const bool eff = frame.can_id & CAN_EFF_FLAG;
    const bool rtr = frame.can_id & CAN_RTR_FLAG;
    if (!eff || rtr) return false; // our wire format is always a 29-bit extended data frame

    const uint32_t id = frame.can_id & CAN_EFF_MASK;

    std::vector<bool> header;
    header.reserve(29);
    for (int i = 28; i >= 0; --i) {
        header.push_back((id >> i) & 1);
    }

    std::vector<uint8_t> payload;
    payload.reserve(frame.can_dlc);
    for (size_t i = 0; i < frame.can_dlc; ++i) {
        payload.push_back(frame.data[i]);
    }

    msg.type = static_cast<MessageType>(SSICanMessageTypeToMessageType[static_cast<SSICanMessageType>(extract_bits(header, 1+4+4, 4))]);
    msg.correlation_id = 0; // Not used in this implementation
    const uint8_t src_nibble = static_cast<uint8_t>(extract_bits(header, 1, 4));
    const uint8_t dest_nibble = static_cast<uint8_t>(extract_bits(header, 1+4, 4));
    msg.src = (src_nibble == 0x0) ? cfg_.sat_src : (0xF0 | src_nibble);
    msg.dest = (dest_nibble == 0x0) ? cfg_.sat_src : (0xF0 | dest_nibble);
    msg.command_or_sensor_id = static_cast<uint16_t>(extract_bits(header, 1+4+4+4, 12));
    msg.channel_hint = ChannelId::Can;
    msg.payload.clear();
    msg.payload.insert(msg.payload.end(), payload.begin(), payload.end());


    return true;
}