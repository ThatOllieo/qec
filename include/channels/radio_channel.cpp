#include "radio_channel.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <iostream>
#include <thread>
#include <vector>

// POSIX + spidev
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>

// gpiod
#include <gpiod.h>

// ---- RFM69 registers & constants (subset) ----
static constexpr uint8_t REG_FIFO           = 0x00;
static constexpr uint8_t REG_OPMODE         = 0x01;
static constexpr uint8_t REG_DATAMODUL      = 0x02;
static constexpr uint8_t REG_BITRATEMSB     = 0x03;
static constexpr uint8_t REG_BITRATELSB     = 0x04;
static constexpr uint8_t REG_FDEVMSB        = 0x05;
static constexpr uint8_t REG_FDEVLSB        = 0x06;
static constexpr uint8_t REG_FRFMSB         = 0x07;
static constexpr uint8_t REG_FRFMID         = 0x08;
static constexpr uint8_t REG_FRFLSB         = 0x09;
static constexpr uint8_t REG_PALEVEL        = 0x11;
static constexpr uint8_t REG_OCP            = 0x13;
static constexpr uint8_t REG_LNA            = 0x18;
static constexpr uint8_t REG_RXBW           = 0x19;
static constexpr uint8_t REG_DIOMAPPING1    = 0x25;
static constexpr uint8_t REG_IRQFLAGS1      = 0x27;
static constexpr uint8_t REG_IRQFLAGS2      = 0x28;
static constexpr uint8_t REG_RSSITHRESH     = 0x29;
static constexpr uint8_t REG_PREAMBLEMSB    = 0x2C;
static constexpr uint8_t REG_PREAMBLELSB    = 0x2D;
static constexpr uint8_t REG_SYNCCONFIG     = 0x2E;
static constexpr uint8_t REG_SYNCVALUE1     = 0x2F;
static constexpr uint8_t REG_SYNCVALUE2     = 0x30;
static constexpr uint8_t REG_PACKETCONFIG1  = 0x37;
static constexpr uint8_t REG_PAYLOADLENGTH  = 0x38;
static constexpr uint8_t REG_FIFOTHRESH     = 0x3C;
static constexpr uint8_t REG_PACKETCONFIG2  = 0x3D;
static constexpr uint8_t REG_TESTPA1        = 0x5A;
static constexpr uint8_t REG_TESTPA2        = 0x5C;
static constexpr uint8_t REG_TESTDAGC       = 0x6F;
static constexpr uint8_t REG_VERSION        = 0x10;

static constexpr uint8_t MODE_SLEEP = 0x00;
static constexpr uint8_t MODE_STDBY = 0x04;
static constexpr uint8_t MODE_FS    = 0x08;
static constexpr uint8_t MODE_TX    = 0x0C;
static constexpr uint8_t MODE_RX    = 0x10;

// --- small helpers ---
static inline void sleep_ms(int ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}
static inline void sleep_us(int us) {
    std::this_thread::sleep_for(std::chrono::microseconds(us));
}

// endian helpers used by your frame format
static inline uint16_t get16(const uint8_t* p) {
    return uint16_t(p[0]) | (uint16_t(p[1]) << 8);
}
static inline void put16(std::vector<uint8_t>& v, uint16_t x) {
    v.push_back(uint8_t(x & 0xFF));
    v.push_back(uint8_t((x >> 8) & 0xFF));
}

// ---------------------------------------------
// SPI + GPIO thin wrappers (mirror your working code)
// ---------------------------------------------
bool spi_transfer(int fd, const uint8_t* tx, uint8_t* rx, size_t len, uint32_t speed_hz) {
    struct spi_ioc_transfer tr {};
    tr.tx_buf = reinterpret_cast<unsigned long>(tx);
    tr.rx_buf = reinterpret_cast<unsigned long>(rx);
    tr.len = static_cast<uint32_t>(len);
    tr.speed_hz = speed_hz;
    tr.bits_per_word = 8;
    tr.delay_usecs = 0;
    return ioctl(fd, SPI_IOC_MESSAGE(1), &tr) >= 0;
}

static inline std::vector<uint8_t> spi_xfer(int fd, const std::vector<uint8_t>& bytes, uint32_t speed_hz) {
    std::vector<uint8_t> rx(bytes.size(), 0);
    if (!spi_transfer(fd, bytes.data(), rx.data(), bytes.size(), speed_hz)) {
        throw std::runtime_error("SPI transfer failed");
    }
    return rx;
}

static inline uint8_t r_read(int fd, uint32_t speed, uint8_t addr) {
    std::vector<uint8_t> tx = { uint8_t(addr & 0x7F), 0x00 };
    auto rx = spi_xfer(fd, tx, speed);
    return rx[1];
}
static inline void r_write(int fd, uint32_t speed, uint8_t addr, uint8_t val) {
    std::vector<uint8_t> tx = { uint8_t(addr | 0x80), uint8_t(val) };
    (void)spi_xfer(fd, tx, speed);
}
static inline void r_burst_write(int fd, uint32_t speed, uint8_t addr, const uint8_t* data, size_t len) {
    std::vector<uint8_t> tx;
    tx.reserve(1 + len);
    tx.push_back(uint8_t(addr | 0x80));
    tx.insert(tx.end(), data, data + len);
    (void)spi_xfer(fd, tx, speed);
}
static inline std::vector<uint8_t> r_burst_read(int fd, uint32_t speed, uint8_t addr, size_t len) {
    std::vector<uint8_t> tx(1 + len, 0x00);
    tx[0] = uint8_t(addr & 0x7F);
    auto rx = spi_xfer(fd, tx, speed);
    return std::vector<uint8_t>(rx.begin() + 1, rx.end());
}

// ---------------------------------------------
// RadioChannel implementation
// ---------------------------------------------

RadioChannel::RadioChannel(RadioConfig cfg) : cfg_(std::move(cfg)) {}
RadioChannel::~RadioChannel() { stop(); }

bool RadioChannel::start() {
    if (running_) return true;
    set_state(ChannelState::Starting);

    if (!hw_open()) {
        set_state(ChannelState::Failed);
        return false;
    }
    if (!hw_configure()) {
        hw_close();
        set_state(ChannelState::Failed);
        return false;
    }

    running_ = true;
    set_state(ChannelState::Running);

    rx_thread_ = std::thread(&RadioChannel::rx_loop, this);
    tx_thread_ = std::thread(&RadioChannel::tx_loop, this);
    return true;
}

void RadioChannel::stop() {
    if (!running_) return;
    running_ = false;

    // nudge TX thread
    tx_frames_.push(std::vector<uint8_t>{});
    if (tx_thread_.joinable()) tx_thread_.join();

    // stop RX thread by switching mode & closing IO
    // (rx_loop will exit when reads fail)
    if (rx_thread_.joinable()) rx_thread_.join();

    hw_close();
    set_state(ChannelState::Stopped);
}

void RadioChannel::send(const CommsMessage& msg) {
    auto frame = serialize(msg);

    // Safety: RFM69 demo configured for <= 65 bytes total (1 len + ≤64 data).
    // Our PHY payload is: [plen][frame-bytes...], where plen = frame.size().
    // Keep a margin: require frame.size() <= 60 bytes.
    if (frame.size() > 60) {
        std::cerr << "[RadioChannel] frame too large for current RFM69 setup ("
                  << frame.size() << " > 60). Implement fragmentation.\n";
        return; // or throw
    }

    // The demo FIFO expects first byte = payload length (<= 64), then payload
    std::vector<uint8_t> phy;
    phy.reserve(1 + frame.size());
    phy.push_back(static_cast<uint8_t>(frame.size()));
    phy.insert(phy.end(), frame.begin(), frame.end());

    tx_frames_.push(std::move(phy));
}

// ---------------------------------------------
// Threads
// ---------------------------------------------

void RadioChannel::tx_loop() {
    while (running_) {
        auto phy = tx_frames_.pop();
        if (!running_) break;
        if (phy.empty()) continue; // nudge

        // Strip the leading length for our helper; hw_tx_packet expects only payload
        if (phy.size() < 1) continue;
        std::vector<uint8_t> payload(phy.begin() + 1, phy.end());

        if (!hw_tx_packet(payload)) {
            std::cerr << "[RadioChannel] TX failed\n";
            set_state(ChannelState::Failed);
            break;
        }
    }
}

void RadioChannel::rx_loop() {
    // Poll RX; each packet returned is already the PHY payload (your frame)
    std::vector<uint8_t> frame;
    while (running_) {
        frame.clear();
        if (!hw_rx_once(frame, 0.100)) { // 100 ms poll
            continue; // timeout/no packet
        }
        // Expect your frame: [LEN(2)] + [TYPE(1)] + [CID(2)] + [SRC(1)] + [DST(1)] + payload
        CommsMessage m{};
        if (parse(frame, m)) {
            if (rx_cb_) rx_cb_(m);
        } else {
            std::cerr << "[RadioChannel] parse failed (size=" << frame.size() << ")\n";
        }
    }
}

// ---------------------------------------------
// Frame format (identical to TcpChannel)
// ---------------------------------------------

std::vector<uint8_t> RadioChannel::serialize(const CommsMessage& m) {
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

bool RadioChannel::parse(const std::vector<uint8_t>& f, CommsMessage& out) {
    if (f.size() < 7) return false;
    uint16_t len = uint16_t(f[0] | (uint16_t(f[1])<<8));
    size_t expect = size_t(len) + 2;
    if (expect != f.size()) return false;

    out.type           = static_cast<MessageType>(f[2]);
    out.correlation_id = uint16_t(f[3] | (uint16_t(f[4])<<8));
    out.src            = f[5];
    out.dest           = f[6];
    out.channel_hint   = ChannelId::Radio;
    out.payload.assign(f.begin()+7, f.end());
    return true;
}

// ---------------------------------------------
// Hardware layer
// ---------------------------------------------

bool RadioChannel::hw_open() {
    // Open SPI
    spi_fd_ = ::open(cfg_.spidev.c_str(), O_RDWR);
    if (spi_fd_ < 0) { perror("open spidev"); return false; }

    uint8_t mode = SPI_MODE_0;
    uint8_t bits = 8;
    if (ioctl(spi_fd_, SPI_IOC_WR_MODE, &mode) < 0) { perror("SPI_IOC_WR_MODE"); return false; }
    if (ioctl(spi_fd_, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) { perror("SPI_IOC_WR_BITS_PER_WORD"); return false; }
    if (ioctl(spi_fd_, SPI_IOC_WR_MAX_SPEED_HZ, &cfg_.spi_speed_hz) < 0) { perror("SPI_IOC_WR_MAX_SPEED_HZ"); return false; }

    // Open GPIO
    gpiod_chip* chip = gpiod_chip_open_by_name("gpiochip0");
    if (!chip) { perror("gpiod_chip_open_by_name"); return false; }

    gpiod_line* reset = gpiod_chip_get_line(chip, cfg_.pin_reset);
    gpiod_line* dio0  = gpiod_chip_get_line(chip, cfg_.pin_dio0);
    if (!reset || !dio0) {
        std::cerr << "gpiod_chip_get_line failed\n";
        gpiod_chip_close(chip);
        return false;
    }
    if (gpiod_line_request_output(reset, "rfm69-reset", 0) < 0) {
        perror("gpiod_line_request_output(reset)");
        gpiod_chip_close(chip);
        return false;
    }
    if (gpiod_line_request_input(dio0, "rfm69-dio0") < 0) {
        perror("gpiod_line_request_input(dio0)");
        gpiod_line_release(reset);
        gpiod_chip_close(chip);
        return false;
    }

    gpio_chip_  = chip;
    gpio_reset_ = reset;
    gpio_dio0_  = dio0;

    // Reset pulse
    gpiod_line_set_value((gpiod_line*)gpio_reset_, 1);
    sleep_ms(10);
    gpiod_line_set_value((gpiod_line*)gpio_reset_, 0);
    sleep_ms(10);
    return true;
}

void RadioChannel::hw_close() {
    // stdby best effort
    try { hw_set_mode(MODE_STDBY); } catch (...) {}

    if (gpio_reset_) { gpiod_line_release((gpiod_line*)gpio_reset_); gpio_reset_ = nullptr; }
    if (gpio_dio0_)  { gpiod_line_release((gpiod_line*)gpio_dio0_);  gpio_dio0_  = nullptr; }
    if (gpio_chip_)  { gpiod_chip_close((gpiod_chip*)gpio_chip_);    gpio_chip_  = nullptr; }

    if (spi_fd_ >= 0) { ::close(spi_fd_); spi_fd_ = -1; }
}

bool RadioChannel::hw_set_mode(uint8_t mode) {
    uint8_t op = r_read(spi_fd_, cfg_.spi_speed_hz, REG_OPMODE);
    r_write(spi_fd_, cfg_.spi_speed_hz, REG_OPMODE, (op & 0xE3) | mode);

    // Wait ModeReady (IRQFLAGS1 bit7)
    auto t0 = std::chrono::steady_clock::now();
    while ((r_read(spi_fd_, cfg_.spi_speed_hz, REG_IRQFLAGS1) & 0x80) == 0) {
        auto dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        if (dt > 1.0) {
            std::cerr << "[RadioChannel] Mode change timeout\n";
            return false;
        }
    }
    return true;
}

static void program_freq(int spi_fd, uint32_t speed, uint32_t freq_hz) {
    // FXOSC=32MHz, Fstep=FXOSC/2^19 ≈ 61.035 Hz
    const double FSTEP = 32'000'000.0 / (1 << 19);
    uint32_t frf = static_cast<uint32_t>(freq_hz / FSTEP);
    r_write(spi_fd, speed, REG_FRFMSB, (frf >> 16) & 0xFF);
    r_write(spi_fd, speed, REG_FRFMID, (frf >> 8)  & 0xFF);
    r_write(spi_fd, speed, REG_FRFLSB, frf & 0xFF);
}

bool RadioChannel::hw_configure() {
    if (!hw_set_mode(MODE_STDBY)) return false;

    // Packet mode, FSK, no shaping
    r_write(spi_fd_, cfg_.spi_speed_hz, REG_DATAMODUL, 0x00);

    // Bitrate
    if (int(cfg_.bitrate) == 4800) {
        r_write(spi_fd_, cfg_.spi_speed_hz, REG_BITRATEMSB, 0x1A);
        r_write(spi_fd_, cfg_.spi_speed_hz, REG_BITRATELSB, 0x0B);
    } else {
        uint16_t br = static_cast<uint16_t>(32'000'000.0 / cfg_.bitrate);
        r_write(spi_fd_, cfg_.spi_speed_hz, REG_BITRATEMSB, (br >> 8) & 0xFF);
        r_write(spi_fd_, cfg_.spi_speed_hz, REG_BITRATELSB, br & 0xFF);
    }

    // FDEV
    uint16_t fdev = static_cast<uint16_t>(cfg_.fdev_hz / (32'000'000.0 / (1 << 19)));
    r_write(spi_fd_, cfg_.spi_speed_hz, REG_FDEVMSB, (fdev >> 8) & 0xFF);
    r_write(spi_fd_, cfg_.spi_speed_hz, REG_FDEVLSB, fdev & 0xFF);

    // Frequency
    program_freq(spi_fd_, cfg_.spi_speed_hz, cfg_.freq_hz);

    // RXBW: (fixed demo value 0b01000001 ≈ 100 kHz)
    r_write(spi_fd_, cfg_.spi_speed_hz, REG_RXBW, 0b01000001);

    // LNA: Zin=200Ω, auto gain
    r_write(spi_fd_, cfg_.spi_speed_hz, REG_LNA, 0b10010000);

    // Sync: enable, size=2, 0x2D, 0xD4
    r_write(spi_fd_, cfg_.spi_speed_hz, REG_SYNCCONFIG, 0x88);
    r_write(spi_fd_, cfg_.spi_speed_hz, REG_SYNCVALUE1, 0x2D);
    r_write(spi_fd_, cfg_.spi_speed_hz, REG_SYNCVALUE2, 0xD4);

    // Preamble
    r_write(spi_fd_, cfg_.spi_speed_hz, REG_PREAMBLEMSB, 0x00);
    r_write(spi_fd_, cfg_.spi_speed_hz, REG_PREAMBLELSB, uint8_t(std::max(0, cfg_.preamble_bytes)));

    // PacketConfig1: CRC on (bit4), variable len (bit7=1), dc-free off
    r_write(spi_fd_, cfg_.spi_speed_hz, REG_PACKETCONFIG1, 0b10011000);

    // Payload length watermark (not fixed length mode)
    r_write(spi_fd_, cfg_.spi_speed_hz, REG_PAYLOADLENGTH, 66);

    // FIFO threshold: TxStart=FifoNotEmpty, thresh=15
    r_write(spi_fd_, cfg_.spi_speed_hz, REG_FIFOTHRESH, 0b10001111);

    // PacketConfig2: AutoRxRestartOn=1
    r_write(spi_fd_, cfg_.spi_speed_hz, REG_PACKETCONFIG2, 0b01000000);

    // DIO0 mapping default (PayloadReady/PacketSent)
    r_write(spi_fd_, cfg_.spi_speed_hz, REG_DIOMAPPING1, 0x00);

    // RSSI threshold coarse
    r_write(spi_fd_, cfg_.spi_speed_hz, REG_RSSITHRESH, 220);

    // PA config
    if (cfg_.pa_rfio) {
        // PA0 (RFIO)
        r_write(spi_fd_, cfg_.spi_speed_hz, REG_OCP, 0x1A);
        r_write(spi_fd_, cfg_.spi_speed_hz, REG_TESTPA1, 0x55);
        r_write(spi_fd_, cfg_.spi_speed_hz, REG_TESTPA2, 0x70);
        int level = std::max(-18, std::min(13, cfg_.power_dbm));
        r_write(spi_fd_, cfg_.spi_speed_hz, REG_PALEVEL, 0b10000000 | (level + 18));
    } else {
        if (cfg_.pa_high) {
            // +20 dBm path
            r_write(spi_fd_, cfg_.spi_speed_hz, REG_OCP, 0x0F);
            r_write(spi_fd_, cfg_.spi_speed_hz, REG_TESTPA1, 0x5D);
            r_write(spi_fd_, cfg_.spi_speed_hz, REG_TESTPA2, 0x7C);
            int level = std::max(5, std::min(20, cfg_.power_dbm));
            r_write(spi_fd_, cfg_.spi_speed_hz, REG_PALEVEL, 0b01100000 | (level + 11));
        } else {
            // PA1 via BOOST (~ +5..+13 dBm)
            r_write(spi_fd_, cfg_.spi_speed_hz, REG_OCP, 0x1A);
            r_write(spi_fd_, cfg_.spi_speed_hz, REG_TESTPA1, 0x55);
            r_write(spi_fd_, cfg_.spi_speed_hz, REG_TESTPA2, 0x70);
            int level = std::max(-2, std::min(13, cfg_.power_dbm));
            r_write(spi_fd_, cfg_.spi_speed_hz, REG_PALEVEL, 0b01000000 | (level + 18));
        }
    }

    // Recommended DAGC
    r_write(spi_fd_, cfg_.spi_speed_hz, REG_TESTDAGC, 0x30);

    // into RX ready state
    return hw_set_mode(MODE_RX);
}

bool RadioChannel::hw_tx_packet(const std::vector<uint8_t>& payload, double timeout_s) {
    if (payload.size() > 64) return false;

    // 1) Force STDBY before touching FIFO
    if (!hw_set_mode(MODE_STDBY)) return false;

    // 2) Write [len][data...] in one burst
    std::vector<uint8_t> frame;
    frame.reserve(1 + payload.size());
    frame.push_back(static_cast<uint8_t>(payload.size()));
    frame.insert(frame.end(), payload.begin(), payload.end());
    r_burst_write(spi_fd_, cfg_.spi_speed_hz, REG_FIFO, frame.data(), frame.size());

    // 3) Go TX and wait for PacketSent
    if (!hw_set_mode(MODE_TX)) return false;

    auto t0 = std::chrono::steady_clock::now();
    while (true) {
        uint8_t irq2 = r_read(spi_fd_, cfg_.spi_speed_hz, REG_IRQFLAGS2);
        if (irq2 & 0x08) break; // PacketSent
        double dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        if (dt > timeout_s) {
            std::cerr << "[RadioChannel] TX timeout\n";
            // optional debug dump here (see below)
            (void)hw_set_mode(MODE_STDBY);
            return false;
        }
        sleep_us(1000);
    }

    // 4) Match demo: TX -> STDBY, then return to RX
    (void)hw_set_mode(MODE_STDBY);
    (void)hw_set_mode(MODE_RX);
    return true;
}

bool RadioChannel::hw_rx_once(std::vector<uint8_t>& out, double timeout_s) {
    // We assume already in RX mode
    auto t0 = std::chrono::steady_clock::now();
    while (true) {
        uint8_t irq2 = r_read(spi_fd_, cfg_.spi_speed_hz, REG_IRQFLAGS2);
        // PayloadReady condition or DIO0 high -> packet present
        if ((irq2 & 0x04) || gpiod_line_get_value((gpiod_line*)gpio_dio0_) == 1) {
            // Check CRC ok (bit1)
            if ((irq2 & 0x02) == 0) {
                // bad CRC → drain one packet: length then bytes
                try {
                    uint8_t length = r_read(spi_fd_, cfg_.spi_speed_hz, REG_FIFO);
                    (void)r_burst_read(spi_fd_, cfg_.spi_speed_hz, REG_FIFO, length);
                } catch (...) {}
                return false;
            }
            uint8_t length = r_read(spi_fd_, cfg_.spi_speed_hz, REG_FIFO);
            auto data = r_burst_read(spi_fd_, cfg_.spi_speed_hz, REG_FIFO, length);
            out.swap(data);
            return true;
        }
        if (timeout_s > 0.0) {
            double dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
            if (dt > timeout_s) return false; // timeout
        }
        sleep_us(1000);
    }
}