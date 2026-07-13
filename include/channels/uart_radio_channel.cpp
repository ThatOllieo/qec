#include "uart_radio_channel.hpp"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <vector>

/* POSIX serial */
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>

/* ── endian helpers (same as RadioChannel / radio_channel.cpp) ─────────── */
static inline uint16_t get16le(const uint8_t* p) {
    return uint16_t(p[0]) | (uint16_t(p[1]) << 8);
}
static inline void put16le(std::vector<uint8_t>& v, uint16_t x) {
    v.push_back(uint8_t(x & 0xFF));
    v.push_back(uint8_t((x >> 8) & 0xFF));
}

/* ── debug helper ───────────────────────────────────────────────────────── */
static std::string hex_dump(const uint8_t* p, size_t n) {
    static const char* hex = "0123456789ABCDEF";
    std::string s;
    s.reserve(n * 3);
    for (size_t i = 0; i < n; ++i) {
        s += hex[p[i] >> 4];
        s += hex[p[i] & 0xF];
        s += ' ';
    }
    return s;
}
static std::string hex_dump(const std::vector<uint8_t>& v) {
    return hex_dump(v.data(), v.size());
}

/* ── baud rate helper ───────────────────────────────────────────────────── */
static speed_t baud_to_speed(uint32_t baud) {
    switch (baud) {
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 921600: return B921600;
        default:
            throw std::invalid_argument(
                "UartRadioChannel: unsupported baud rate " + std::to_string(baud));
    }
}

/* ────────────────────────────────────────────────────────────────────────── */

UartRadioChannel::UartRadioChannel(UartRadioConfig cfg)
    : cfg_(std::move(cfg)) {}

UartRadioChannel::~UartRadioChannel() { stop(); }

/* ── IChannel::start ────────────────────────────────────────────────────── */
bool UartRadioChannel::start() {
    std::cerr << "[UartRadioChannel] start() device=" << cfg_.device
              << " baud=" << cfg_.baud << "\n";

    if (running_) {
        std::cerr << "[UartRadioChannel] start() called while already running — ignored\n";
        return true;
    }
    set_state(ChannelState::Starting);

    if (!hw_open()) {
        std::cerr << "[UartRadioChannel] start() aborted — hw_open failed\n";
        set_state(ChannelState::Failed);
        return false;
    }

    running_ = true;
    set_state(ChannelState::Running);

    rx_thread_ = std::thread(&UartRadioChannel::rx_loop, this);
    tx_thread_ = std::thread(&UartRadioChannel::tx_loop, this);
    std::cerr << "[UartRadioChannel] start() ok, rx/tx threads launched\n";
    return true;
}

/* ── IChannel::stop ─────────────────────────────────────────────────────── */
void UartRadioChannel::stop() {
    if (!running_) return;
    running_ = false;

    /* Unblock tx_loop's TSQueue::pop() with a sentinel empty frame */
    tx_frames_.push(std::vector<uint8_t>{});

    if (tx_thread_.joinable()) tx_thread_.join();

    /* Closing the fd unblocks the blocking read() in rx_loop */
    hw_close();

    if (rx_thread_.joinable()) rx_thread_.join();

    set_state(ChannelState::Stopped);
}

/* ── IChannel::send ─────────────────────────────────────────────────────── */
bool UartRadioChannel::send(const CommsMessage& msg) {
    std::cerr << "[UartRadioChannel] send() type=" << int(msg.type)
              << " corr=" << msg.correlation_id
              << " src=" << int(msg.src) << " dest=" << int(msg.dest)
              << " payload=" << msg.payload.size() << "B state="
              << int(state_.load(std::memory_order_relaxed)) << "\n";

    if (state_.load(std::memory_order_relaxed) != ChannelState::Running) {
        std::cerr << "[UartRadioChannel] send() on non-running channel — dropped\n";
        return false;
    }

    auto frame = serialize(msg);

    if (frame.size() > cfg_.max_frame_len) {
        std::cerr << "[UartRadioChannel] frame too large ("
                  << frame.size() << " > " << cfg_.max_frame_len
                  << ") — dropped\n";
        return false;
    }

    /* Prepend 2-byte LE length prefix to form a UART wire frame.
     * The RP2354 bridge's uart_poll_frame() expects exactly this layout:
     *   [len_lo][len_hi][frame bytes...]                                  */
    std::vector<uint8_t> wire;
    wire.reserve(2 + frame.size());
    wire.push_back(uint8_t(frame.size() & 0xFF));
    wire.push_back(uint8_t((frame.size() >> 8) & 0xFF));
    wire.insert(wire.end(), frame.begin(), frame.end());

    std::cerr << "[UartRadioChannel] send() queuing " << wire.size()
              << "B: " << hex_dump(wire) << "\n";

    tx_frames_.push(std::move(wire));
    return true;
}

/* ── TX thread ──────────────────────────────────────────────────────────── */
void UartRadioChannel::tx_loop() {
    std::cerr << "[UartRadioChannel] tx_loop started\n";
    try {
        while (running_) {
            auto wire = tx_frames_.pop();
            if (!running_) break;        /* sentinel or stop() called */
            if (wire.empty()) continue;  /* sentinel nudge */

            std::cerr << "[UartRadioChannel] tx_loop writing " << wire.size()
                      << "B to fd=" << uart_fd_ << "\n";

            if (!write_frame(wire)) {
                std::cerr << "[UartRadioChannel] write_frame failed — channel Failed\n";
                set_state(ChannelState::Failed);
                break;
            }
            std::cerr << "[UartRadioChannel] tx_loop wrote " << wire.size() << "B ok\n";
        }
    } catch (const std::exception& e) {
        std::cerr << "[UartRadioChannel] tx_loop exception: " << e.what() << "\n";
        set_state(ChannelState::Failed);
    }
    std::cerr << "[UartRadioChannel] tx_loop exiting\n";
}

/* ── RX thread ──────────────────────────────────────────────────────────── */
/*
 * Reassembly state machine — mirrors uart_poll_frame() on the RP2354 but
 * blocking-read style (select() with timeout so running_ can be rechecked).
 *
 * Each UART wire frame:
 *   [len_lo : 1][len_hi : 1][frame bytes : len]
 *
 * 'frame bytes' is what serialize() produced on this side originally, so
 * parse() can consume it directly.
 */
void UartRadioChannel::rx_loop() {
    enum class RxState { LenLo, LenHi, Body };

    RxState  state     = RxState::LenLo;
    uint16_t expected  = 0;
    uint8_t  len_lo    = 0;
    std::vector<uint8_t> frame_buf;
    frame_buf.reserve(256);

    auto read_byte = [&](uint8_t& out) -> bool {
        /* Use select() with a 100ms timeout so we can notice running_=false */
        while (running_) {
            fd_set fds;
            FD_ZERO(&fds);
            FD_SET(uart_fd_, &fds);
            struct timeval tv { .tv_sec = 0, .tv_usec = 100000 };
            int r = select(uart_fd_ + 1, &fds, nullptr, nullptr, &tv);
            if (r < 0) {
                if (errno == EINTR) continue;
                std::cerr << "[UartRadioChannel] rx_loop select() error: "
                          << strerror(errno) << "\n";
                return false;   /* fd closed or error — stop */
            }
            if (r == 0) continue; /* timeout — recheck running_ */

            ssize_t n = ::read(uart_fd_, &out, 1);
            if (n == 1) return true;
            if (n == 0 || (n < 0 && errno != EINTR)) {
                std::cerr << "[UartRadioChannel] rx_loop read() returned n=" << n
                          << (n < 0 ? std::string(" errno=") + strerror(errno) : "")
                          << " — stopping rx\n";
                return false;
            }
        }
        return false;
    };

    std::cerr << "[UartRadioChannel] rx_loop started, listening on fd=" << uart_fd_ << "\n";

    try {
        uint8_t b;
        while (running_) {
            if (!read_byte(b)) break;

            std::cerr << "[UartRadioChannel] rx_loop byte=0x"
                      << hex_dump(&b, 1) << "state="
                      << (state == RxState::LenLo ? "LenLo" :
                          state == RxState::LenHi ? "LenHi" : "Body") << "\n";

            switch (state) {
                case RxState::LenLo:
                    len_lo = b;
                    state  = RxState::LenHi;
                    break;

                case RxState::LenHi: {
                    expected = uint16_t(len_lo) | (uint16_t(b) << 8);
                    frame_buf.clear();
                    if (expected == 0 || expected > cfg_.max_frame_len) {
                        std::cerr << "[UartRadioChannel] bad frame length "
                                  << expected << " — resyncing\n";
                        state = RxState::LenLo;
                    } else {
                        std::cerr << "[UartRadioChannel] rx_loop expecting "
                                  << expected << "B frame\n";
                        frame_buf.reserve(expected);
                        state = RxState::Body;
                    }
                    break;
                }

                case RxState::Body:
                    frame_buf.push_back(b);
                    if (frame_buf.size() == expected) {
                        std::cerr << "[UartRadioChannel] rx_loop frame complete: "
                                  << hex_dump(frame_buf) << "\n";
                        /* Full frame received — parse and dispatch */
                        CommsMessage msg{};
                        if (parse(frame_buf, msg)) {
                            std::cerr << "[UartRadioChannel] rx_loop parsed type="
                                      << int(msg.type) << " corr=" << msg.correlation_id
                                      << " src=" << int(msg.src) << " dest=" << int(msg.dest)
                                      << " payload=" << msg.payload.size() << "B"
                                      << " cb=" << (rx_cb_ ? "set" : "NULL") << "\n";
                            if (rx_cb_) rx_cb_(msg);
                        } else {
                            std::cerr << "[UartRadioChannel] parse failed ("
                                      << frame_buf.size() << " bytes)\n";
                        }
                        state = RxState::LenLo;
                    }
                    break;
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "[UartRadioChannel] rx_loop exception: " << e.what() << "\n";
        set_state(ChannelState::Failed);
    }
    std::cerr << "[UartRadioChannel] rx_loop exiting\n";
}

/* ── Hardware layer ─────────────────────────────────────────────────────── */
bool UartRadioChannel::hw_open() {
    uart_fd_ = ::open(cfg_.device.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (uart_fd_ < 0) {
        std::cerr << "[UartRadioChannel] open(" << cfg_.device << "): "
                  << strerror(errno) << "\n";
        return false;
    }
    std::cerr << "[UartRadioChannel] opened " << cfg_.device << " fd=" << uart_fd_ << "\n";

    struct termios tty{};
    if (tcgetattr(uart_fd_, &tty) != 0) {
        std::cerr << "[UartRadioChannel] tcgetattr: " << strerror(errno) << "\n";
        hw_close();
        return false;
    }

    speed_t spd;
    try {
        spd = baud_to_speed(cfg_.baud);
    } catch (const std::invalid_argument& e) {
        std::cerr << "[UartRadioChannel] " << e.what() << "\n";
        hw_close();
        return false;
    }

    cfsetospeed(&tty, spd);
    cfsetispeed(&tty, spd);

    /* 8N1, no flow control, raw mode — set field-by-field (mirrors the
     * known-good uart_send reference) rather than via cfmakeraw() */
    tty.c_cflag &= ~PARENB;        // no parity
    tty.c_cflag &= ~CSTOPB;        // 1 stop bit
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;            // 8 data bits
    tty.c_cflag &= ~CRTSCTS;       // no hardware flow control
    tty.c_cflag |= CREAD | CLOCAL; // enable receiver, ignore modem lines

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ECHOE;
    tty.c_lflag &= ~ISIG;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;

    /* Non-blocking read handled via select() in rx_loop — VMIN=0, VTIME=0 */
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(uart_fd_, TCSANOW, &tty) != 0) {
        std::cerr << "[UartRadioChannel] tcsetattr: " << strerror(errno) << "\n";
        hw_close();
        return false;
    }

    tcflush(uart_fd_, TCIOFLUSH);  /* discard any stale bytes from before open */
    std::cerr << "[UartRadioChannel] hw_open configured ok baud=" << cfg_.baud << "\n";
    return true;
}

void UartRadioChannel::hw_close() {
    if (uart_fd_ >= 0) {
        std::cerr << "[UartRadioChannel] closing fd=" << uart_fd_ << "\n";
        ::close(uart_fd_);
        uart_fd_ = -1;
    }
}

/* Write all bytes, retrying on short writes and EINTR */
bool UartRadioChannel::write_frame(const std::vector<uint8_t>& wire) {
    const uint8_t* ptr = wire.data();
    size_t remaining   = wire.size();

    while (remaining > 0) {
        ssize_t n = ::write(uart_fd_, ptr, remaining);
        if (n < 0) {
            if (errno == EINTR) continue;
            std::cerr << "[UartRadioChannel] write: " << strerror(errno) << "\n";
            return false;
        }
        std::cerr << "[UartRadioChannel] write_frame wrote " << n << "/"
                  << remaining << "B this call\n";
        ptr       += n;
        remaining -= static_cast<size_t>(n);
    }
    return true;
}

/* ── Wire format (verbatim from RadioChannel) ────────────────────────────── */
std::vector<uint8_t> UartRadioChannel::serialize(const CommsMessage& m) {
    const uint16_t body_len = uint16_t(1 + 2 + 1 + 1 + 2 + m.payload.size());
    std::vector<uint8_t> f;
    f.reserve(2 + body_len);
    put16le(f, body_len);
    f.push_back(static_cast<uint8_t>(m.type));
    put16le(f, m.correlation_id);
    f.push_back(m.src);
    f.push_back(m.dest);
    put16le(f, m.command_or_sensor_id);
    f.insert(f.end(), m.payload.begin(), m.payload.end());
    return f;
}

bool UartRadioChannel::parse(const std::vector<uint8_t>& f, CommsMessage& out) {
    if (f.size() < 9) return false;

    uint16_t body_len = get16le(f.data());
    if (size_t(body_len) + 2 != f.size()) return false;

    out.type                  = static_cast<MessageType>(f[2]);
    out.correlation_id        = get16le(f.data() + 3);
    out.src                   = f[5];
    out.dest                  = f[6];
    out.command_or_sensor_id  = get16le(f.data() + 7);
    out.channel_hint          = ChannelId::Uart;
    out.payload.assign(f.begin() + 9, f.end());
    return true;
}
