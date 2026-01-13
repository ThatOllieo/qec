#include <iostream>
#include <cstring>
#include <array>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>

#include "udp_channel.hpp"

UdpChannel::UdpChannel(UdpConfig cfg) : cfg_(std::move(cfg)){}
UdpChannel::~UdpChannel(){stop();}

bool UdpChannel::start(){
    if(running_) return true;
    set_state(ChannelState::Starting);
    if(!open_and_bind()){
        set_state(ChannelState::Failed);
        return false;
    }

    init_dest_table();

    running_ = true;
    set_state(ChannelState::Running);

    rx_thread_ = std::thread(&UdpChannel::rx_loop, this);
    tx_thread_ = std::thread(&UdpChannel::tx_loop, this);
    return true;
}

void UdpChannel::stop(){
    if(!running_) return;
    running_ = false;

    tx_items_.push(TxItem{});
    if(tx_thread_.joinable()) tx_thread_.join();
    close_sock();
    if(rx_thread_.joinable()) rx_thread_.join();

    set_state(ChannelState::Stopped);
}

bool UdpChannel::open_and_bind(){
    sock_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if(sock_ < 0){std::perror("socket error"); return false;}

    int one = 1;
    ::setsockopt(sock_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    #ifdef SO_REUSEPORT
    ::setsockopt(sock_, SOL_SOCKET, SO_REUSEPORT, &one, sizeof(one));
    #endif

    timeval tv{};
    tv.tv_sec = 0;
    tv.tv_usec = 200 * 1000; // 200ms
    ::setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    sockaddr_in local{};
    local.sin_family = AF_INET;
    local.sin_port = htons(5000);
    local.sin_addr.s_addr = htonl(INADDR_ANY);

    if(::bind(sock_, (sockaddr*)&local, sizeof(local)) < 0){
        std::perror("bind error");
        ::close(sock_); sock_ = -1; return false;
    }
    return true;
}

void UdpChannel::init_dest_table() {
    // Precompute 10.42.0.X:5000 for all 256 possible destination IDs.
    // NOTE: In a /24, .0 and .255 are traditionally network/broadcast; reserve if you need.
    for (int d = 0; d < 256; ++d) {
        sockaddr_in a{};
        a.sin_family = AF_INET;
        a.sin_port = htons(5000);

        // 10.42.0.d
        uint32_t ip = (10u << 24) | (42u << 16) | (0u << 8) | uint32_t(d);
        a.sin_addr.s_addr = htonl(ip);

        dest_addrs_[size_t(d)] = a;
    }
}

void UdpChannel::close_sock(){
    if(sock_ >= 0){
        ::shutdown(sock_, SHUT_RDWR);
        ::close(sock_);
        sock_ = -1;
    }
}

void UdpChannel::send(const CommsMessage& msg){
    TxItem item;
    item.frame = serialize(msg);
    item.dest  = dest_addrs_[msg.dest];
    tx_items_.push(std::move(item));
}

void UdpChannel::tx_loop() {
    while (running_) {
        auto item = tx_items_.pop(); // blocks
        if (!running_) break;
        if (item.frame.empty()) continue; // nudge frame

        // UDP datagram: one send = one packet
        ssize_t r = ::sendto(sock_, item.frame.data(), item.frame.size(), 0,
                             (sockaddr*)&item.dest, sizeof(item.dest));
        if (r < 0) {
            std::perror("[UdpChannel] sendto");
            set_state(ChannelState::Failed);
            break;
        }
        if (size_t(r) != item.frame.size()) {
            std::cerr << "[UdpChannel] partial send? sent=" << r
                      << " expected=" << item.frame.size() << "\n";
            // Generally, UDP sendto is all-or-nothing; treat as failure.
            set_state(ChannelState::Failed);
            break;
        }
    }
}

void UdpChannel::rx_loop() {
    // Choose a buffer size that comfortably fits your max message.
    // UDP max is 65507 payload for IPv4, but fragmentation is bad.
    // If you keep your messages <= ~1200 bytes you're happiest.
    std::vector<uint8_t> buf(2048);

    while (running_) {
        sockaddr_in sender{};
        socklen_t sender_len = sizeof(sender);

        ssize_t n = ::recvfrom(sock_, buf.data(), buf.size(), 0,
                               (sockaddr*)&sender, &sender_len);

        if (n < 0) {
            // With SO_RCVTIMEO you will get EAGAIN/EWOULDBLOCK periodically.
            if (!running_) break;
            continue;
        }
        if (n == 0) {
            // UDP "0 bytes" is unusual but possible; ignore.
            continue;
        }

        std::vector<uint8_t> f(buf.begin(), buf.begin() + n);

        // Debug: show sender + LEN bytes if present
        if (f.size() >= 2) {
            uint8_t lenbuf[2]{ f[0], f[1] };
            uint16_t body_len = get16(lenbuf);

            std::cerr << "[UdpChannel] from " << inet_ntoa(sender.sin_addr)
                      << ":" << ntohs(sender.sin_port)
                      << " LEN bytes: 0x" << std::hex << int(lenbuf[0])
                      << " 0x" << int(lenbuf[1])
                      << " (little-endian body_len=" << std::dec << body_len << ")\n";
        } else {
            std::cerr << "[UdpChannel] datagram too small (" << f.size() << ")\n";
            continue;
        }

        CommsMessage m{};
        if (parse(f, m)) {
            if (on_receive_) on_receive_(m);
        } else {
            std::cerr << "[UdpChannel] parse failed, dropping datagram (size=" << f.size() << ")\n";
        }
    }
}

std::vector<uint8_t> UdpChannel::serialize(const CommsMessage& m) {
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

bool UdpChannel::parse(const std::vector<uint8_t>& f, CommsMessage& out) {
    if (f.size() < 7) return false;

    const size_t len = uint16_t(f[0] | (uint16_t(f[1]) << 8));
    const size_t expect = len + 2;
    if (expect != f.size()) return false;

    out.type           = static_cast<MessageType>(f[2]);
    out.correlation_id = uint16_t(f[3] | (uint16_t(f[4]) << 8));
    out.src            = f[5];
    out.dest           = f[6];
    out.channel_hint   = ChannelId::Wifi;

    out.payload.assign(f.begin() + 7, f.end());
    return true;
}