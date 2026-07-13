// can_test.cpp
//
// Standalone SocketCAN test — setup, transmit, receive.
// Deliberately has no dependency on IChannel/TSQueue: just the raw
// socket calls, so you can see exactly what a CanChannel wrapper needs
// to do underneath before folding it into that shape.
//
// Build:
//   g++ -std=c++17 -O2 -Wall -o can_test can_test.cpp
//
// Bring the interface up first:
//   sudo ip link set can0 up type can bitrate 500000
//
// Run:
//   ./can_test rx        -- unfiltered receive loop, prints every frame
//   ./can_test tx         -- sends one extended-ID (29-bit) test frame
//
// To see them talk to each other, run `./can_test rx` in one terminal
// and `./can_test tx` in another (or `cansend`/`candump` from can-utils
// against either side).

#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <linux/can.h>
#include <linux/can/raw.h>

namespace {

// Open + bind a raw CAN socket to the named interface (e.g. "can0").
// Returns -1 on failure (error already printed via perror).
int open_can_socket(const std::string& ifname) {
    int fd = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd < 0) {
        std::perror("socket(PF_CAN, SOCK_RAW, CAN_RAW)");
        return -1;
    }

    struct ifreq ifr {};
    std::strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ - 1);
    if (::ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
        std::fprintf(stderr, "ioctl(SIOCGIFINDEX) on %s: %s\n",
                     ifname.c_str(), std::strerror(errno));
        ::close(fd);
        return -1;
    }

    struct sockaddr_can addr {};
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (::bind(fd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::perror("bind");
        ::close(fd);
        return -1;
    }

    // Explicit catch-all filter: mask=0 means "don't care about any ID
    // bit," so this matches every ID, standard or extended, data or RTR.
    // Functionally identical to skipping setsockopt entirely — it's here
    // so the "unfiltered on purpose" intent is visible in code, not just
    // implied by an absent call.
    struct can_filter filt {};
    filt.can_id   = 0;
    filt.can_mask = 0;
    if (::setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FILTER, &filt, sizeof(filt)) < 0) {
        std::perror("setsockopt(CAN_RAW_FILTER)");
        ::close(fd);
        return -1;
    }

    // Also surface bus errors (bus-off, error-passive, etc.) as frames
    // with CAN_ERR_FLAG set, instead of them being silently swallowed.
    can_err_mask_t err_mask = CAN_ERR_MASK;
    if (::setsockopt(fd, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask, sizeof(err_mask)) < 0) {
        std::perror("setsockopt(CAN_RAW_ERR_FILTER)");
        // not fatal — continue without error-frame visibility
    }

    return fd;
}

void print_frame(const struct can_frame& frame) {
    if (frame.can_id & CAN_ERR_FLAG) {
        std::printf("[ERR ] class=0x%08X\n", frame.can_id & CAN_ERR_MASK);
        return;
    }

    const bool eff = frame.can_id & CAN_EFF_FLAG;
    const bool rtr = frame.can_id & CAN_RTR_FLAG;
    const uint32_t id = frame.can_id & (eff ? CAN_EFF_MASK : CAN_SFF_MASK);

    std::printf("[%s%s] id=0x%0*X dlc=%d data=",
                eff ? "EFF" : "SFF", rtr ? " RTR" : "",
                eff ? 8 : 3, id, frame.can_dlc);

    if (!rtr) {
        for (int i = 0; i < frame.can_dlc; ++i) {
            std::printf("%02X ", frame.data[i]);
        }
    }
    std::printf("\n");
}

void run_rx(int fd) {
    std::printf("listening on fd=%d (unfiltered) — Ctrl+C to stop\n", fd);
    struct can_frame frame {};
    while (true) {
        ssize_t n = ::read(fd, &frame, sizeof(frame));
        if (n < 0) {
            std::perror("read");
            break;
        }
        if (n != sizeof(frame)) {
            std::fprintf(stderr, "short read: %zd bytes (expected %zu)\n",
                         n, sizeof(frame));
            continue;
        }
        print_frame(frame);
    }
}

void run_tx(int fd) {
    struct can_frame frame {};

    // 29-bit extended ID: set CAN_EFF_FLAG, ID goes in the low 29 bits.
    // Swap this for your actual custom-header encoding once you're
    // wiring real protocol logic in on top of this.
    frame.can_id  = 0x1ABCDE01 | CAN_EFF_FLAG;
    frame.can_dlc = 8;
    const uint8_t payload[8] = {0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x01, 0x02, 0x03};
    std::memcpy(frame.data, payload, sizeof(payload));

    ssize_t n = ::write(fd, &frame, sizeof(frame));
    if (n != sizeof(frame)) {
        std::perror("write");
        return;
    }
    std::printf("sent: ");
    print_frame(frame);
}

}  // namespace

int main(int argc, char** argv) {
    const std::string ifname = "can0";
    const std::string mode = argc > 1 ? argv[1] : "";

    if (mode != "tx" && mode != "rx") {
        std::fprintf(stderr, "usage: %s [tx|rx]\n", argv[0]);
        return 1;
    }

    int fd = open_can_socket(ifname);
    if (fd < 0) return 1;

    if (mode == "tx") {
        run_tx(fd);
    } else {
        run_rx(fd);
    }

    ::close(fd);
    return 0;
}