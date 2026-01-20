// gs main.cpp
#include <iostream>
#include <thread>
#include <unordered_map>
#include <functional>
#include <vector>
#include <string>
#include <cctype>

#include <nlohmann/json.hpp>
using json = nlohmann::json;

#include "../include/tsqueue.hpp"
#include "../include/events.hpp"
#include "../include/comms_manager.hpp"
#include "../include/channels/radio_channel.hpp"
#include "../include/channels/udp_channel.hpp"

#include "../include/ws_link.hpp"

static std::unordered_map<uint16_t, std::function<void(const std::vector<uint8_t>&)>> telem_plans;
static std::unordered_map<uint16_t, std::function<void()>>                            cmd_plans;

static uint16_t parse_u16(const std::string &s) {
    if (s.rfind("0x", 0) == 0 || s.rfind("0X", 0) == 0)
        return static_cast<uint16_t>(std::stoul(s, nullptr, 16));
    return static_cast<uint16_t>(std::stoul(s, nullptr, 10));
}
static uint8_t parse_u8(const std::string &s) {
    return static_cast<uint8_t>(parse_u16(s));
}

int main() {
    TSQueue<Event> eventList;

    CommsManager comms(eventList);
    comms.set_src(0x01);

    UdpConfig udpcfg;
    auto udp = std::make_unique<UdpChannel>(udpcfg);
    comms.register_channel(std::move(udp));

    RadioConfig rcfg;
    rcfg.freq_hz   = 434'000'000;
    rcfg.pa_high   = true;
    rcfg.power_dbm = 17;
    rcfg.rx_bw_khz = 100.0;
    rcfg.bitrate   = 55'000.0;
    rcfg.fdev_hz   = 50'000.0;
    rcfg.pin_reset = 22;            
    rcfg.pin_dio0  = 25;            
    rcfg.spidev    = "/dev/spidev0.1";

    auto radio = std::make_unique<RadioChannel>(rcfg);
    comms.register_channel(std::move(radio));

    if (!comms.start()) {
        std::cerr << "[GS] Comms start failed\n";
        return 1;
    }

    WSLink wslink(eventList);
    wslink.start(9002);

    std::atomic<bool> running{true};

    constexpr uint16_t toPoll[] = {0x0003, 0x0001};
    std::thread polling([&]{
        while(running){

            for (uint16_t x : toPoll) {
                uint16_t corr = comms.request_telem_async(
                    0xEF,
                    ChannelId::Wifi,
                    x, // sensor id
                    std::chrono::milliseconds(1000), // timeout
                    2                                 // retries
                );
                if (!corr) {
                    std::cout << "[GS] failed to send poll telemetry request\n";
                    continue;
                }

                telem_plans[corr] = [corr,&wslink,x](const std::vector<uint8_t>& bytes){
                    for (auto b : bytes) std::cout << std::hex << int(b) << ' ';
                    std::cout << std::dec << "\n";

                    json j = {
                        {"type", "telemetry"},
                        {"sensor", x},
                        {"data", bytes}
                    };
                    wslink.broadcast(j.dump());

                };
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5000));
        }
    });

    std::thread ev_thread([&]{
        while (running) {
            Event e = eventList.pop(); 
            switch (e.type) {
                case EventType::TelemetryArrived: {
                    
                    auto &a = std::get<EvTelemetryArrived>(e.data);

                    if (auto it = telem_plans.find(a.correlation_id); it != telem_plans.end()) {
                        it->second(a.bytes);
                        telem_plans.erase(it);
                    } else {
                        std::cout << "[GS] Untracked TelemetryArrived corr=" << a.correlation_id
                                  << " size=" << a.bytes.size() << "\n";
                    }
                    break;
                }
                case EventType::TelemetryFailed: {
                    auto &f = std::get<EvTelemetryFailed>(e.data);
                    std::cout << "[GS] Telemetry timeout/error corr=" << f.correlation_id << "\n";
                    telem_plans.erase(f.correlation_id);
                    break;
                }
                case EventType::CommandAcked: {
                    auto &c = std::get<EvCommandAcked>(e.data);
                    if (auto it = cmd_plans.find(c.correlation_id); it != cmd_plans.end()) {
                        it->second();
                        cmd_plans.erase(it);
                    } else {
                        std::cout << "[GS] Untracked CommandAcked corr=" << c.correlation_id << "\n";
                    }
                    break;
                }
                case EventType::CommandFailed: {
                    auto &c = std::get<EvCommandFailed>(e.data);
                    std::cout << "[GS] Command timeout/error corr=" << c.correlation_id << "\n";
                    cmd_plans.erase(c.correlation_id);
                    break;
                }

                case EventType::TelemetryRequest: {
                    auto &t = std::get<EvTelemetryRequest>(e.data);
                    std::cout << "[GS] Inbound TelemetryRequest (sensor=" << t.sensor_id
                              << ", corr=" << t.correlation_id << ") — ignoring in GS demo\n";
                    break;
                }
                case EventType::Command: {
                    auto &c = std::get<EvCommand>(e.data);
                    std::cout << "[GS] Inbound Command (cmd=0x" << std::hex << c.command_id
                              << std::dec << ", corr=" << c.correlation_id << ") — ignoring but forwarding to UI in GS demo\n";

                    json j = {
                        {"type", "command"},
                        {"command_id", c.command_id},
                    };
                    wslink.broadcast(j.dump());

                    break;
                }

                default:
                    break;
            }
        }
    });

    // --- Simple interactive shell ---
    std::cout <<
        "GS radio console:\n"
        "  tlm <dest> <sensor>      e.g. tlm 0xEF 0x0003\n"
        "  cmd <dest> <cmdid>       e.g. cmd 0xEF 0x0011\n"
        "  quit/exit\n";

    std::string line;
    while (running && std::cout << "gs> " && std::getline(std::cin, line)) {
        // Trim
        while (!line.empty() && std::isspace(static_cast<unsigned char>(line.back()))) line.pop_back();
        if (line.empty()) continue;

        // Tokenize
        std::vector<std::string> tok;
        {
            std::string cur;
            for (char ch : line) {
                if (std::isspace(static_cast<unsigned char>(ch))) {
                    if (!cur.empty()) { tok.push_back(cur); cur.clear(); }
                } else cur.push_back(ch);
            }
            if (!cur.empty()) tok.push_back(cur);
        }
        if (tok.empty()) continue;

        std::string cmd = tok[0];
        for (auto &c : cmd) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));

        if (cmd == "quit" || cmd == "exit") {
            break;
        } else if (cmd == "tlm") {
            if (tok.size() < 3) {
                std::cout << "usage: tlm <dest> <sensor>\n";
                continue;
            }
            uint8_t  dest   = parse_u8(tok[1]);
            uint16_t sensor = parse_u16(tok[2]);

            uint16_t corr = comms.request_telem_async(
                dest,
                ChannelId::Wifi,
                sensor,
                std::chrono::milliseconds(1000), // timeout
                2                                 // retries
            );
            if (!corr) {
                std::cout << "[GS] failed to send telemetry request\n";
                continue;
            }
            std::cout << "[GS] Sent TLM corr=" << corr << " dest=0x" << std::hex << int(dest)
                      << " sensor=0x" << sensor << std::dec << "\n";
            telem_plans[corr] = [corr,&wslink,sensor](const std::vector<uint8_t>& bytes){
                std::cout << "[GS] TelemetryArrived corr=" << corr
                          << " len=" << bytes.size() << " data=";
                for (auto b : bytes) std::cout << std::hex << int(b) << ' ';
                std::cout << std::dec << "\n";

                json j = {
                    {"type", "telemetry"},
                    {"sensor", sensor},
                    {"data", bytes}
                };
                wslink.broadcast(j.dump());

            };

        } else if (cmd == "cmd") {
            if (tok.size() < 3) {
                std::cout << "usage: cmd <dest> <cmdid>\n";
                continue;
            }
            uint8_t  dest  = parse_u8(tok[1]);
            uint16_t cmdid = parse_u16(tok[2]);

            uint16_t corr = comms.send_command_async(
                dest,
                ChannelId::Wifi,
                cmdid,
                /*args*/{},
                std::chrono::milliseconds(1000),
                1
            );
            if (!corr) {
                std::cout << "[GS] failed to send command\n";
                continue;
            }
            std::cout << "[GS] Sent CMD corr=" << corr << " dest=0x" << std::hex << int(dest)
                      << " cmd=0x" << cmdid << std::dec << "\n";

            cmd_plans[corr] = [corr](){
                std::cout << "[GS] CommandAcked corr=" << corr << "\n";
            };

        } else {
            std::cout << "unknown: " << cmd << "\n";
        }
    }

    running = false;
    comms.stop();
    wslink.stop();
    if (ev_thread.joinable()) ev_thread.join();
    if(polling.joinable()) polling.join();

    return 0;
}