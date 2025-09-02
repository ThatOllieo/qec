#include "tcp_channel.hpp"
#include <iostream>
#include <cstring>
#include <chrono>
#include <algorithm>

// POSIX sockets
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <unistd.h>

TcpChannel::TcpChannel(TcpConfig cfg) : cfg_(std::move(cfg)) {}
TcpChannel::~TcpChannel() { stop(); }

bool TcpChannel::start() {
    if (running_) return true;
    set_state(ChannelState::Starting);
    if (!connect_once()) {
        set_state(ChannelState::Failed);
        return false;
    }
    running_ = true;
    set_state(ChannelState::Running);

    rx_thread_ = std::thread(&TcpChannel::rx_loop, this);
    tx_thread_ = std::thread(&TcpChannel::tx_loop, this);
    return true;
}

void TcpChannel::stop() {
    if (!running_) return;
    running_ = false;

    // nudge TX thread if blocked
    tx_frames_.push(std::vector<uint8_t>{});
    if (tx_thread_.joinable()) tx_thread_.join();

    // shutting down socket wakes RX
    close_sock();
    if (rx_thread_.joinable()) rx_thread_.join();

    set_state(ChannelState::Stopped);
}

void TcpChannel::send(const CommsMessage& msg) {
    tx_frames_.push(serialize(msg));
}

bool TcpChannel::connect_once() {
    sock_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (sock_ < 0) { std::perror("socket"); return false; }

    int one = 1;
    ::setsockopt(sock_, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

    sockaddr_in addr{}; addr.sin_family = AF_INET;
    addr.sin_port = htons(cfg_.port);
    if (::inet_pton(AF_INET, cfg_.host.c_str(), &addr.sin_addr) != 1) {
        std::cerr << "inet_pton failed for " << cfg_.host << "\n";
        ::close(sock_); sock_ = -1; return false;
    }
    if (::connect(sock_, (sockaddr*)&addr, sizeof(addr)) < 0) {
        std::perror("connect");
        ::close(sock_); sock_ = -1; return false;
    }
    return true;
}

void TcpChannel::close_sock() {
    if (sock_ >= 0) {
        ::shutdown(sock_, SHUT_RDWR);
        ::close(sock_);
        sock_ = -1;
    }
}

bool TcpChannel::write_all(const uint8_t* data, size_t n) {
    size_t sent = 0;
    while (sent < n) {
        ssize_t r = ::send(sock_, data + sent, n - sent, 0);
        if (r <= 0) return false;
        sent += size_t(r);
    }
    return true;
}

bool TcpChannel::read_exact(uint8_t* data, size_t n) {
    size_t got = 0;
    while (got < n) {
        ssize_t r = ::recv(sock_, data + got, n - got, 0);
        if (r <= 0) return false; // peer closed or error
        got += size_t(r);
    }
    return true;
}

void TcpChannel::tx_loop() {
    while (running_) {
        auto frame = tx_frames_.pop(); // blocks
        if (!running_) break;
        if (frame.empty()) continue;   // nudge frame

        if (!write_all(frame.data(), frame.size())) {
            std::cerr << "[TcpChannel] send failed, closing socket\n";
            set_state(ChannelState::Failed);
            close_sock();
            break;
        }
    }
}

void TcpChannel::rx_loop() {
    while (running_) {
        uint8_t lenbuf[2];
        if (!read_exact(lenbuf, 2)) {
            std::cerr << "[TcpChannel] peer closed or read error on LEN\n";
            set_state(ChannelState::Failed);
            break;
        }
        uint16_t body_len = get16(lenbuf);

        // Debug: print raw LEN bytes and interpreted value
        std::cerr << "[TcpChannel] LEN bytes: 0x"
                  << std::hex << int(lenbuf[0]) << " 0x" << int(lenbuf[1])
                  << "  (little-endian body_len=" << std::dec << body_len << ")\n";

        if (body_len < 5) { // must be at least TYPE(1)+CID(2)+SRC(1)+DST(1)
            std::cerr << "[TcpChannel] invalid length (<6)\n";
            set_state(ChannelState::Failed);
            break;
        }
        if (body_len > 65535 - 2) {
            std::cerr << "[TcpChannel] absurd length, dropping\n";
            set_state(ChannelState::Failed);
            break;
        }

        std::vector<uint8_t> body(body_len);
        if (!read_exact(body.data(), body.size())) {
            std::cerr << "[TcpChannel] short read on body (" << body_len << ")\n";
            set_state(ChannelState::Failed);
            break;
        }

        // Optional: show first few body bytes
        if (!body.empty()) {
            std::cerr << "[TcpChannel] Body[0..min]:";
            for (size_t i = 0; i < std::min<size_t>(body.size(), 8); ++i)
                std::cerr << " " << std::hex << int(body[i]);
            std::cerr << std::dec << "\n";
        }

        std::vector<uint8_t> f; f.reserve(2 + body.size());
        f.push_back(lenbuf[0]); f.push_back(lenbuf[1]);
        f.insert(f.end(), body.begin(), body.end());

        CommsMessage m{};
        if (parse(f, m)) {
            if (on_receive_) on_receive_(m);
        } else {
            std::cerr << "[TcpChannel] parse failed, dropping frame (size=" << f.size() << ")\n";
        }
    }
}

std::vector<uint8_t> TcpChannel::serialize(const CommsMessage& m) {
    const uint16_t body_len = uint16_t(1 + 2 + 1 + 1 + m.payload.size());
    std::vector<uint8_t> f; f.reserve(2 + body_len);
    put16(f, body_len);
    f.push_back(static_cast<uint8_t>(m.type));
    put16(f, m.correlation_id);
    f.push_back(m.src);
    f.push_back(m.dest);
    f.insert(f.end(), m.payload.begin(), m.payload.end());
    return f;
}

bool TcpChannel::parse(const std::vector<uint8_t>& f, CommsMessage& out) {
    if (f.size() < 7) return false;
    const size_t len = uint16_t(f[0] | (uint16_t(f[1])<<8));
    const size_t expect = len + 2;
    if (expect != f.size()) return false;

    out.type           = static_cast<MessageType>(f[2]);
    out.correlation_id = uint16_t(f[3] | (uint16_t(f[4])<<8));
    out.src            = f[5];
    out.dest           = f[6];
    out.channel_hint   = ChannelId::Wifi; 
    
    out.payload.assign(f.begin()+7, f.end());
    return true;
}