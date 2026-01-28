// gs main.cpp
#include <iostream>
#include <thread>
#include <unordered_map>
#include <functional>
#include <vector>
#include <string>
#include <cctype>
#include <atomic>
#include <mutex>

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
static std::mutex plans_mtx;

static uint16_t parse_u16(const std::string &s) {
    if (s.rfind("0x", 0) == 0 || s.rfind("0X", 0) == 0)
        return static_cast<uint16_t>(std::stoul(s, nullptr, 16));
    return static_cast<uint16_t>(std::stoul(s, nullptr, 10));
}
static uint8_t parse_u8(const std::string &s) {
    return static_cast<uint8_t>(parse_u16(s));
}

void cmdLineOut(const std::string &msg, WSLink &wslink) {
    std::cout << msg << std::endl;
    json j = {
        {"type", "console"},
        {"message", msg}
    };
    wslink.broadcast(j.dump());
}

// Returns false if the user requested to quit, true otherwise.
bool cmdLineParse(CommsManager &comms, WSLink &wslink, const std::string &line) {
    // Tokenize
    std::vector<std::string> tok;
    {
        std::string cur;
        for (char ch : line) {
            if (std::isspace(static_cast<unsigned char>(ch))) {
                if (!cur.empty()) { tok.push_back(cur); cur.clear(); }
            } else {
                cur.push_back(ch);
            }
        }
        if (!cur.empty()) tok.push_back(cur);
    }
    if (tok.empty()) return true;

    std::string cmd = tok[0];
    for (auto &c : cmd) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));



    if (cmd == "quit" || cmd == "exit") {
        return false;
    } else if (cmd == "tlm") {
        if (tok.size() < 3) {
            cmdLineOut("usage: tlm <dest> <sensor>", wslink);
            return true;
        }
        uint8_t  dest   = parse_u8(tok[1]);
        uint16_t sensor = parse_u16(tok[2]);

        uint8_t hint = static_cast<uint8_t>(ChannelId::Wifi);
        if(tok.size() >= 4){
            hint = parse_u8(tok[3]);
        }

        uint16_t corr = comms.request_telem_async(
            dest,
            static_cast<ChannelId>(hint),
            sensor,
            std::chrono::milliseconds(1000), // timeout
            2                                 // retries
        );
        if (!corr) {
            cmdLineOut("[GS] failed to send telemetry request", wslink);
            return true;
        }
        
        cmdLineOut("[GS] Sent TLM corr=" + std::to_string(corr) +
                   " dest=" + std::to_string(int(dest)) +
                   " sensor=" + std::to_string(sensor), wslink);

        {
            std::lock_guard<std::mutex> lock(plans_mtx);
            telem_plans[corr] = [corr, &wslink, sensor](const std::vector<uint8_t> &bytes) {
                cmdLineOut("[GS] TelemetryArrived corr=" + std::to_string(corr) +
                           " len=" + std::to_string(bytes.size()), wslink);
                for (auto b : bytes) std::cout << std::hex << int(b) << ' ';
                std::cout << std::dec << "\n";

                json j = {
                    {"type", "telemetry"},
                    {"sensor", sensor},
                    {"data", bytes}
                };
                wslink.broadcast(j.dump());
            };
        }

        return true;

    } else if (cmd == "cmd") {
        if (tok.size() < 3) {
            cmdLineOut("usage: cmd <dest> <cmdid>", wslink);
            return true;
        }
        uint8_t  dest  = parse_u8(tok[1]);
        uint16_t cmdid = parse_u16(tok[2]);

        uint8_t hint = static_cast<uint8_t>(ChannelId::Wifi);
        if(tok.size() >= 4){
            hint = parse_u8(tok[3]);
        }

        uint16_t corr = comms.send_command_async(
            dest,
            static_cast<ChannelId>(hint),
            cmdid,
            /*args*/{},
            std::chrono::milliseconds(1000),
            1
        );
        if (!corr) {
            cmdLineOut("[GS] failed to send command", wslink);
            return true;
        }
        cmdLineOut("[GS] Sent CMD corr=" + std::to_string(corr) +
                   " dest=" + std::to_string(int(dest)) +
                   " cmd=" + std::to_string(cmdid), wslink);

        {
            std::lock_guard<std::mutex> lock(plans_mtx);
            cmd_plans[corr] = [corr, &wslink]() {
                cmdLineOut("[GS] CommandAcked corr=" + std::to_string(corr), wslink);
            };
        }

        return true;

    } else {
        cmdLineOut("unknown command: " + cmd, wslink);
        return true;
    }
}

int main(int argc, char* argv[]) {
    bool cmdLineMode = true;
    for(int i = 1; i < argc; ++i){
        if(argv[i] == std::string("--service")){
            cmdLineMode = false;
        }
    }

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
                    cmdLineOut("[GS] failed to send poll telemetry request", wslink);
                    continue;
                }

                {
                    std::lock_guard<std::mutex> lock(plans_mtx);
                    telem_plans[corr] = [corr, &wslink, x](const std::vector<uint8_t>& bytes) {
                        cmdLineOut("[GS] Polled TelemetryArrived corr=" + std::to_string(corr) +
                                   " len=" + std::to_string(bytes.size()), wslink);

                        json j = {
                            {"type", "telemetry"},
                            {"sensor", x},
                            {"data", bytes}
                        };
                        wslink.broadcast(j.dump());

                    };
                }
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

                    std::function<void(const std::vector<uint8_t>&)> cb;
                    {
                        std::lock_guard<std::mutex> lock(plans_mtx);
                        if (auto it = telem_plans.find(a.correlation_id); it != telem_plans.end()) {
                            cb = std::move(it->second);
                            telem_plans.erase(it);
                        }
                    }
                    if (cb) {
                        cb(a.bytes);
                    } else {
                        cmdLineOut("[GS] Untracked TelemetryArrived corr=" + std::to_string(a.correlation_id) +
                                   " size=" + std::to_string(a.bytes.size()), wslink);
                    }
                    break;
                }
                case EventType::TelemetryFailed: {
                    auto &f = std::get<EvTelemetryFailed>(e.data);
                    cmdLineOut("[GS] Telemetry timeout/error corr=" + std::to_string(f.correlation_id), wslink);
                    {
                        std::lock_guard<std::mutex> lock(plans_mtx);
                        telem_plans.erase(f.correlation_id);
                    }
                    break;
                }
                case EventType::CommandAcked: {
                    auto &c = std::get<EvCommandAcked>(e.data);
                    std::function<void()> cb;
                    {
                        std::lock_guard<std::mutex> lock(plans_mtx);
                        if (auto it = cmd_plans.find(c.correlation_id); it != cmd_plans.end()) {
                            cb = std::move(it->second);
                            cmd_plans.erase(it);
                        }
                    }
                    if (cb) {
                        cb();
                    } else {
                        cmdLineOut("[GS] Untracked CommandAcked corr=" + std::to_string(c.correlation_id), wslink);
                    }
                    break;
                }
                case EventType::CommandFailed: {
                    auto &c = std::get<EvCommandFailed>(e.data);
                    cmdLineOut("[GS] Command timeout/error corr=" + std::to_string(c.correlation_id) +
                               " reason=" + c.reason, wslink);
                    {
                        std::lock_guard<std::mutex> lock(plans_mtx);
                        cmd_plans.erase(c.correlation_id);
                    }
                    break;
                }

                case EventType::TelemetryRequest: {
                    auto &t = std::get<EvTelemetryRequest>(e.data);
                    cmdLineOut("[GS] Inbound TelemetryRequest (sensor=" + std::to_string(t.sensor_id) +
                               ", corr=" + std::to_string(t.correlation_id) + ") — ignoring in GS demo", wslink);
                    break;
                }
                case EventType::Command: {
                    auto &c = std::get<EvCommand>(e.data);
                    cmdLineOut("[GS] Inbound Command (cmd=0x" + std::to_string(c.command_id) +
                               ", corr=" + std::to_string(c.correlation_id) + ") — ignoring in GS demo", wslink);
                    json j = {
                        {"type", "command"},
                        {"command_id", c.command_id},
                    };
                    wslink.broadcast(j.dump());

                    break;
                }
                case EventType::WSMessageReceived: {
                    auto &wsc = std::get<EvWSMessageReceived>(e.data);

                    json j;
                    try {
                        j = json::parse(wsc.jsonText);
                    } catch (const json::parse_error& e) {
                        std::cerr << "JSON parse error: " << e.what() << "\n";
                        cmdLineOut("Invalid WS in: JSON parse error", wslink);
                        break;
                    }

                    if( !j.contains("type") || !j["type"].is_string() ) {
                        std::cerr << "Invalid WS in: missing or invalid 'type' field\n";
                        cmdLineOut("Invalid WS in: missing or invalid 'type' field", wslink);
                        break;
                    }
                    std::string type = j["type"];
                    if(type == "console"){
                        if(!j.contains("message") || !j["message"].is_string()) {
                            std::cerr << "Invalid console WS console command: missing or invalid 'message' field\n";
                            cmdLineOut("Invalid WS console command: missing or invalid 'message' field", wslink);
                            break;
                        }
                        cmdLineParse(comms, wslink, j.at("message").get<std::string>());
                    }
                    
                    break;
                }

                default:
                    break;
            }
        }
    });

    if (cmdLineMode) {
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

            if (!cmdLineParse(comms, wslink, line)) break;
        }
    } else {
        cmdLineOut("[GS] Running in service mode", wslink);
        while (running) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

    running = false;
    comms.stop();
    wslink.stop();
    if (ev_thread.joinable()) ev_thread.join();
    if(polling.joinable()) polling.join();

    return 0;
}