#include <iostream>
#include <variant>
#include <thread>
#include <unordered_map>
#include <functional>
#include "camera_service.hpp"
#include "../include/tsqueue.hpp"
#include "../include/events.hpp"
#include "../include/deployment_watcher.hpp"
#include "../include/imu.hpp"
#include "../include/comms_manager.hpp"
#include "../include/channels/udp_channel.hpp"
#include "../include/channels/radio_channel.hpp"
#include "utils.hpp"

#include "../include/exceptions.hpp"
#include "../include/module_states.hpp"
#include "../include/logger.hpp"

#include <fstream>
#include <string>
#include <sys/wait.h>

//Quick demo function to get the temperature of the cm5's cpu
float getCPUTemperature() {
    std::ifstream file("/sys/class/thermal/thermal_zone0/temp");
    if (!file.is_open()) {
        throw QecException(
            ErrorCode::IOError,
            ErrorSeverity::Recoverable,
            "Unable to open CPU temperature file",
            "getCPUTemperature",
            {{"path","/sys/class/thermal/thermal_zone0/temp"}}
        );
    }

    std::string tempStr;
    if(!std::getline(file, tempStr)){
        throw QecException(
            ErrorCode::IOError,
            ErrorSeverity::Recoverable,
            "Failed to read CPU temperature file",
            "getCPUTemperature",
            {{"path","/sys/class/thermal/thermal_zone0/temp"}}
        );
    }
    file.close();

    // Convert the temperature to degrees Celsius
    try{
        return std::stof(tempStr) / 1000.0f;
    }
    catch(const std::exception& ex){
        throw QecException(
            ErrorCode::ProtocolError,
            ErrorSeverity::Recoverable,
            "CPU temperature file contained invalid data",
            "getCPUTemperature",
            {{"raw_value",tempStr}},
            ex.what()
        );
    }
}

// --- Logging/Helper functions ---
static void logStartupFailure(const char* module, const char* action, const QecException& ex) {
    std::cerr << severityTag(ex.severity())
              << '[' << module << "] "
              << action << ": "
              << ex.what() << '\n';
}

// Run an scp shell-out once, returning true only if the process actually exited 0.
static bool runScpOnce(const std::string& cmd) {
    int status = system(cmd.c_str());
    if (status == -1) return false;
    return WIFEXITED(status) && WEXITSTATUS(status) == 0;
}

// Light reliability fix: one retry after a short backoff before giving up.
static bool runScpWithRetry(const std::string& cmd) {
    if (runScpOnce(cmd)) return true;
    logLine(ErrorSeverity::Warning, "MAIN", "scp failed, retrying once: " + cmd);
    std::this_thread::sleep_for(std::chrono::milliseconds(750));
    return runScpOnce(cmd);
}

//function called that gets the data for each sensor id
static std::vector<uint8_t> gather_telem_bytes(uint16_t sensor_id, DeploymentWatcher& deploy, IMU& imu) {
    try{
        switch (sensor_id)
        {
        //deploy switch
        case 0x0001:
            return {static_cast<uint8_t>(deploy.getState())};
            break;
        //cpu temp
        case 0x0003: {
            std::vector<uint8_t> pkt;
            float temp = getCPUTemperature();
            pkt.reserve(sizeof(float));
            appendBytes(pkt, temp);
            return pkt;
        }

        //imu calibration status
        case 0x2010: {
            IMU::CalibrationStatus calibstat = imu.getCalibrationStatus();
            return {calibstat.sys,calibstat.gyro,calibstat.accel,calibstat.mag}; }
            break;
        //imu euler angles
        case 0x2020: {
            if (!imu.getCalibrationStatus().fullyCalibrated()) {
                logLine(ErrorSeverity::Warning, "MAIN", "Serving IMU telemetry (sensor_id=0x2020) while uncalibrated");
            }
            std::vector<uint8_t> pkt;
            IMU::EulerAngles eulang = imu.getEulerAngles();
            pkt.reserve(3 * sizeof(float));
            appendBytes(pkt, eulang.heading);
            appendBytes(pkt, eulang.roll);
            appendBytes(pkt, eulang.pitch);
            return pkt;
            break;
        }

        //imu quaternion
        case 0x2030: {
            if (!imu.getCalibrationStatus().fullyCalibrated()) {
                logLine(ErrorSeverity::Warning, "MAIN", "Serving IMU telemetry (sensor_id=0x2030) while uncalibrated");
            }
            std::vector<uint8_t> pkt;
            IMU::Quaternion quat = imu.getQuaternion();
            pkt.reserve(4 * sizeof(float));
            appendBytes(pkt, quat.w);
            appendBytes(pkt, quat.x);
            appendBytes(pkt, quat.y);
            appendBytes(pkt, quat.z);
            return pkt;
            break;
        }

        //imu gravity vector
        case 0x2040: {
            if (!imu.getCalibrationStatus().fullyCalibrated()) {
                logLine(ErrorSeverity::Warning, "MAIN", "Serving IMU telemetry (sensor_id=0x2040) while uncalibrated");
            }
            std::vector<uint8_t> pkt;
            IMU::Vector3 grav = imu.getGravity();
            pkt.reserve(3 * sizeof(float));
            appendBytes(pkt, grav.x);
            appendBytes(pkt, grav.y);
            appendBytes(pkt, grav.z);
            return pkt;
            break;
        }

        //imu linear accel.
        case 0x2050:{
            if (!imu.getCalibrationStatus().fullyCalibrated()) {
                logLine(ErrorSeverity::Warning, "MAIN", "Serving IMU telemetry (sensor_id=0x2050) while uncalibrated");
            }
            std::vector<uint8_t> pkt;
            IMU::Vector3 linacc = imu.getLinearAccel();
            pkt.reserve(3 * sizeof(float));
            appendBytes(pkt, linacc.x);
            appendBytes(pkt, linacc.y);
            appendBytes(pkt, linacc.z);
            return pkt;
            break;
        }

        //fallback (deadbeef). Just a default, can be used as a detection of no sensor found or just to test the connection is working.
        default:
            return { uint8_t(sensor_id & 0xFF), uint8_t(sensor_id >> 8), 0xDE, 0xAD, 0xBE, 0xEF };
            break;
        }
    }
    catch(const QecException&){
        throw;
    }
    catch(const std::exception& ex){
        throw QecException(
            ErrorCode::IOError,
            ErrorSeverity::Recoverable,
            "Failed to gather telemetry bytes",
            "gather_telem_bytes",
            {{"sensor_id", std::to_string(sensor_id)}},
            ex.what()
        );
    }
}

//handler for incoming commands
int handleCommand(const EvCommand& c, CommsManager& comms, CameraModule& cams, IMU& imu,
                   std::unordered_map<uint16_t, std::function<void()>>& cmd_plans) {
    try{
        switch (c.command_id)
        {
        //OBC CONTROLS 0x0***
        //basic, connection test
        case 0x00:
            comms.reply_ok_command(c.correlation_id);
            break;
        //poweroff
        case 0x01:
            comms.reply_ok_command(c.correlation_id);
            system("sudo poweroff");
            break;
        //reboot
        case 0x02:
            comms.reply_ok_command(c.correlation_id);
            system("sudo reboot");
            break;

        //manual image and imu capture trigger
        case 0x11:
            cams.take_both("/home/pi/left.jpg","/home/pi/right.jpg", 42);
            imu.triggerCapture(5000);
            comms.reply_ok_command(c.correlation_id);
            break;

        //send saved images
        case 0x12:
            setTimeout([c, &comms, &cmd_plans]() {
                bool ok = runScpWithRetry("scp /home/pi/left.jpg /home/pi/right.jpg pi@10.42.0.1:/var/www/juk/media/");
                if (!ok) {
                    logLine(ErrorSeverity::Recoverable, "MAIN", "scp failed after retry (cmd 0x12)");
                    comms.reply_err_command(c.correlation_id);
                    return;
                }
                comms.reply_ok_command(c.correlation_id);
                // Command request demo (cmd 0x0011, no args)
                uint16_t corr2 = comms.send_command_async(
                    /*dest*/0x01,
                    ChannelId::Wifi,
                    /*cmd*/0x0013,
                    /*args*/{},
                    std::chrono::milliseconds(1000),
                    1
                );
                if (corr2) {
                    std::cout << "[MAIN] Sent Command corr=" << corr2 << "\n";
                    cmd_plans[corr2] = [](){
                        std::cout << "[MAIN] Command acknowledged successfully!\n";
                    };
                }
            }, std::chrono::milliseconds(1));
            break;

        //COMMS MANAGER CONTROLS 0x1***
        //restart WiFi channel
        case 0x1011:
            setTimeout([c, &comms]() {
                comms.restart_channel(ChannelId::Wifi);
                comms.reply_ok_command(c.correlation_id);
            }, std::chrono::milliseconds(200));
            break;

        //start WiFi channel
        case 0x1012:
            setTimeout([c, &comms]() {
                if(comms.start_channel(ChannelId::Wifi)){
                    comms.reply_ok_command(c.correlation_id);
                }
                else{
                    comms.reply_err_command(c.correlation_id);
                }
            }, std::chrono::milliseconds(200));
            break;

        //stop WiFi channel
        case 0x1013:
            setTimeout([c, &comms]() {
                comms.stop_channel(ChannelId::Wifi);
                comms.reply_ok_command(c.correlation_id);
            }, std::chrono::milliseconds(200));
            break;

        //drops unrecognised command.
        default:
            comms.reply_err_command(c.correlation_id);
            std::cout << "[MAIN] unrecognised command" << std::endl;
            break;
        }
    }
    catch(const QecException&){
        throw;
    }
    catch(const std::exception& ex){
        throw CommsError(
            ErrorCode::StateError,
            ErrorSeverity::Recoverable,
            "Command handling failed",
            "handleCommand",
            {
                {"command_id", std::to_string(c.command_id)},
                {"correlation_id", std::to_string(c.correlation_id)}
            },
            ex.what()
        );
    }

    return 0;
}


int run() {
    // create main event list, this is the core part of this software, everything that happens is submitted as an event to this list
    // for the main thread to act upon
    TSQueue<Event> eventList;

    // Pending async plans for tracking replies (when this file makes requests and sends commands)
    std::unordered_map<uint16_t, std::function<void(const std::vector<uint8_t>&)>> telem_plans;
    std::unordered_map<uint16_t, std::function<void()>> cmd_plans;

    //camera setup, note that the events list is passed by reference so that events can be added by the camera thread. e.g. ive taken a photo
    CameraModule cams(eventList);


    CameraModuleConfig cfg;
    cfg.left_index = 0;
    cfg.right_index = 1;
    cfg.width = 3840;
    cfg.height = 2160;
    cfg.warmup_frames = 16;
    cfg.jpeg_quality = 85;

    /*
    CameraModuleConfig cfg;
    cfg.left_index = 0;
    cfg.right_index = 1;
    cfg.width = 1280;
    cfg.height = 720;
    cfg.warmup_frames = 16;
    cfg.jpeg_quality = 50;
    */

    try {
        cams.startup(cfg);
    }
    catch (const CamsError& ex) {
        logStartupFailure("CAMS", "Failed to start cameras", ex);
        if (ex.severity() == ErrorSeverity::Fatal) { return 1; }
    }
    catch (const std::exception& ex) {
        std::cerr << "[FATAL][CAMS] Failed to start cameras with std::exception: " << ex.what() << '\n';
        return 1;
    }
    catch (...) {
        std::cerr << "[FATAL][CAMS] Failed to start cameras with unknown exception\n";
        return 1;
    }

    //Deployment switch, configured so that on release of the switch, an event is pushed to the main list
    DeploymentWatcher deploy(eventList);
    try {
        deploy.start();
    }
    catch (const DeployWatchError& ex) {
        logStartupFailure("DEPLOY", "Failed to start deployment watcher", ex);
        if (ex.severity() == ErrorSeverity::Fatal) { return 1; }
    }
    catch (const std::exception& ex) {
        std::cerr << "[FATAL][DEPLOY] Failed to start deployment watcher with std::exception: " << ex.what() << '\n';
        return 1;
    }
    catch (...) {
        std::cerr << "[FATAL][DEPLOY] Failed to start deployment watcher with unknown exception\n";
        return 1;
    }

    //Imu is started, required for the IMU telemetry requests or other onboard use
    IMU imu(eventList);
    try {
        imu.start();
    }
    catch (const IMUError& ex) {
        logStartupFailure("IMU", "Failed to start IMU", ex);
        if (ex.severity() == ErrorSeverity::Fatal) { return 1; }
    }
    catch (const std::exception& ex) {
        std::cerr << "[FATAL][IMU] Failed to start IMU with std::exception: " << ex.what() << '\n';
        return 1;
    }
    catch (...) {
        std::cerr << "[FATAL][IMU] Failed to start IMU with unknown exception\n";
        return 1;
    }

    //Comms manager is what handles all off board communications, incoming messages become events in the main list, and main thread can
    // simply call functions to request data or send commands elsewhere.
    CommsManager comms(eventList);
    const uint8_t sat_src = 0xEF; // this device's identifier, reused below for the heartbeat broadcast
    comms.set_src(sat_src);

    //TcpConfig tcpcfg;
    //tcpcfg.host = "10.42.0.1";
    //tcpcfg.port = 5000;
    //auto tcp = std::make_unique<TcpChannel>(tcpcfg);
    //comms.register_channel(std::move(tcp));

    try {
        UdpConfig udpcfg;
        auto udp = std::make_unique<UdpChannel>(udpcfg);
        comms.register_channel(std::move(udp));
    }
    catch (const CommsError& ex) {
        logStartupFailure("COMMS", "Failed to setup and register UDP channel", ex);
        if (ex.severity() == ErrorSeverity::Fatal) { return 1; }
    }
    catch (const std::exception& ex) {
        std::cerr << "[FATAL][COMMS] Failed to setup and register UDP channel with std::exception: " << ex.what() << '\n';
        return 1;
    }
    catch (...) {
        std::cerr << "[FATAL][COMMS] Failed to setup and register UDP channel with unknown exception\n";
        return 1;
    }

    try {
        RadioConfig rcfg;
        rcfg.freq_hz = 434'000'000;
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
    }
    catch (const CommsError& ex) {
        logStartupFailure("COMMS", "Failed to setup and register RADIO channel", ex);
        if (ex.severity() == ErrorSeverity::Fatal) { return 1; }
    }
    catch (const std::exception& ex) {
        std::cerr << "[FATAL][COMMS] Failed to setup and register RADIO channel with std::exception: " << ex.what() << '\n';
        return 1;
    }
    catch (...) {
        std::cerr << "[FATAL][COMMS] Failed to setup and register RADIO channel with unknown exception\n";
        return 1;
    }

    try {
        comms.start();
    }
    catch (const CommsError& ex) {
        logStartupFailure("COMMS", "Failed to start comms manager", ex);
        if (ex.severity() == ErrorSeverity::Fatal) { return 1; }
    }
    catch (const std::exception& ex) {
        std::cerr << "[FATAL][COMMS] Failed to start comms manager with std::exception: " << ex.what() << '\n';
        return 1;
    }
    catch (...) {
        std::cerr << "[FATAL][COMMS] Failed to start comms manager with unknown exception\n";
        return 1;
    }

    // Periodic health heartbeat: sleeps and posts a synthetic event into the main
    // queue, so the actual broadcast send stays on the main thread like everything else.
    // Wrapped so a single failure never kills the thread.
    std::thread([&eventList]() {
        for (;;) {
            try {
                std::this_thread::sleep_for(std::chrono::seconds(30));
                Event e; e.type = EventType::HeartbeatTick; e.data = EvHeartbeatTick{};
                eventList.push(std::move(e));
            } catch (const std::exception& ex) {
                logLine(ErrorSeverity::Recoverable, "HEARTBEAT", std::string("heartbeat thread error: ") + ex.what());
            } catch (...) {
                logLine(ErrorSeverity::Recoverable, "HEARTBEAT", "heartbeat thread unknown error");
            }
        }
    }).detach();

    const auto process_start = std::chrono::steady_clock::now();


    // --- DEMO outbound requests: fire once on startup ---
    /*
    {
        // Telemetry request demo (sensor 7)
        uint16_t corr = comms.request_telem_async(
            0x01,
            ChannelId::Wifi,
            7,
            std::chrono::milliseconds(1000),
            2
        );
        if (corr) {
            std::cout << "[MAIN] Sent TelemetryRequest corr=" << corr << "\n";
            telem_plans[corr] = [](const std::vector<uint8_t>& bytes) {
                std::cout << "[MAIN] Got " << bytes.size() << " telemetry bytes: ";
                for (auto b : bytes) std::cout << std::hex << int(b) << ' ';
                std::cout << std::dec << "\n";
            };
        }
    }
    */

    uint64_t seq = 1; //counter, used for logging
    for (;;) {
        Event e = eventList.pop(); // blocks until an event arrives
        e.seq = seq++;

        try{
            switch (e.type) {
                //on satellite deployment switch trigger, cameras are told to take photos
                case EventType::DeploymentTriggered: {
                    const auto& d = std::get<EvDeploymentTriggered>(e.data);
                    std::cout << "[MAIN] DEPLOY" << std::endl;

                    if(cams.state() == ModuleState::Running){
                        try{
                            cams.take_both("/home/pi/left.jpg","/home/pi/right.jpg", 42);
                        }
                        catch (const CamsError& ex) {
                            std::cerr << severityTag(ex.severity()) << "[CAMS] " << ex.what() << '\n';
                        }
                    }

                    imu.triggerCapture(5000);
                    break;
                }

                case EventType::MotionCaptured:{
                    const auto& m = std::get<EvMotionCaptured>(e.data);

                    if (!m.ok) {
                        logLine(ErrorSeverity::Warning, "MAIN", "Skipping SCP - IMU capture failed: " + m.error);
                        break;
                    }

                    std::string path = m.path;
                    setTimeout([path, &comms, &cmd_plans](){
                        std::cout << "[MAIN] Got mocap event, for path " << path << std::endl;
                        bool ok = runScpWithRetry(std::string("scp ") + path + " pi@10.42.0.1:/var/www/juk/startup.csv");
                        if (!ok) {
                            logLine(ErrorSeverity::Recoverable, "MAIN", "scp failed after retry (MotionCaptured)");
                            return;
                        }

                        uint16_t corr2 = comms.send_command_async(
                            /*dest*/0x01,
                            ChannelId::Wifi,
                            /*cmd*/0x0014,
                            /*args*/{},
                            std::chrono::milliseconds(1000),
                            1
                        );
                        if (corr2) {
                            std::cout << "[MAIN] Sent Command corr=" << corr2 << "\n";
                            cmd_plans[corr2] = [](){
                                std::cout << "[MAIN] Command acknowledged successfully!\n";
                            };
                        }

                    }, std::chrono::milliseconds(1));
                    break;
                }

                //on photo taken, these are automatically sent to the groundstation address
                case EventType::PhotoTaken: {
                    const auto& p = std::get<EvPhotoTaken>(e.data);
                    std::cout << "[MAIN] (seq " << e.seq << ") PhotoTaken: " << p.path << " ok=" << p.ok << "\n";

                    if (!p.ok) {
                        logLine(ErrorSeverity::Warning, "MAIN", "Skipping SCP - photo capture failed: " + p.error);
                        break;
                    }

                    setTimeout([p, &comms, &cmd_plans]() {
                        std::cout << "[MAIN] triggering scp send\n";
                        bool ok = runScpWithRetry("scp /home/pi/left.jpg /home/pi/right.jpg pi@10.42.0.1:/var/www/juk/media/");
                        if (!ok) {
                            logLine(ErrorSeverity::Recoverable, "MAIN", "scp failed after retry (PhotoTaken)");
                            return;
                        }
                        // Command request demo (cmd 0x0011, no args)
                        uint16_t corr2 = comms.send_command_async(
                            /*dest*/0x01,
                            ChannelId::Wifi,
                            /*cmd*/0x0013,
                            /*args*/{},
                            std::chrono::milliseconds(1000),
                            1
                        );
                        if (corr2) {
                            std::cout << "[MAIN] Sent Command corr=" << corr2 << "\n";
                            cmd_plans[corr2] = [](){
                                std::cout << "[MAIN] Command acknowledged successfully!\n";
                            };
                        }
                    }, std::chrono::milliseconds(1));

                    break;
                }
                // on command recieve, handle
                case EventType::Command: {
                    auto& c = std::get<EvCommand>(e.data);
                    std::cout << "[MAIN] Cmd CorrId: " << c.correlation_id << std::endl;
                    handleCommand(c, comms, cams, imu, cmd_plans);
                    break;

                }
                //on telemetry request, get telem for specifed sensor, reply
                case EventType::TelemetryRequest: {
                    auto& t = std::get<EvTelemetryRequest>(e.data);
                    std::cout << "[MAIN] Tlm CorrId: " << t.correlation_id << " sensorid: " << t.sensor_id << std::endl;
                    auto bytes = gather_telem_bytes(t.sensor_id,deploy,imu);
                    comms.reply_telem(t.correlation_id, bytes);
                    break;
                }

                //if telemtry is recieved (and its something we've requested, do *something*)
                case EventType::TelemetryArrived: {
                    auto& a = std::get<EvTelemetryArrived>(e.data);
                    if (auto it = telem_plans.find(a.correlation_id); it != telem_plans.end()) {
                        it->second(a.bytes);
                        telem_plans.erase(it);
                    }
                    break;
                }
                // telemtry request timeout or got an error
                case EventType::TelemetryFailed: {
                    auto& f = std::get<EvTelemetryFailed>(e.data);
                    std::cout << "[MAIN] telem timeout corr=" << f.correlation_id << "\n";
                    telem_plans.erase(f.correlation_id);
                    break;
                }
                // command has been acknowledged
                case EventType::CommandAcked: {
                    auto& c2 = std::get<EvCommandAcked>(e.data);
                    if (auto it = cmd_plans.find(c2.correlation_id); it != cmd_plans.end()) {
                        it->second();
                        cmd_plans.erase(it);
                    }
                    break;
                }
                // command failed or errored
                case EventType::CommandFailed: {
                    auto& c2 = std::get<EvCommandFailed>(e.data);
                    std::cout << "[MAIN] cmd timeout corr=" << c2.correlation_id << "\n";
                    cmd_plans.erase(c2.correlation_id);
                    break;
                }
                // background worker or channel reported a structural failure
                case EventType::ModuleFailed: {
                    const auto& mf = std::get<EvModuleFailed>(e.data);
                    logLine(mf.severity, "MAIN", "module='" + mf.module + "' reason='" + mf.reason + "'");
                    break;
                }
                // periodic health broadcast
                case EventType::HeartbeatTick: {
                    std::vector<uint8_t> payload;

                    float temp = -1.0f;
                    try { temp = getCPUTemperature(); }
                    catch (const std::exception& ex) {
                        logLine(ErrorSeverity::Warning, "MAIN", std::string("heartbeat: temp read failed: ") + ex.what());
                    }
                    appendBytes(payload, temp);

                    payload.push_back(static_cast<uint8_t>(cams.state()));
                    payload.push_back(static_cast<uint8_t>(imu.state()));
                    payload.push_back(static_cast<uint8_t>(deploy.getState()));
                    payload.push_back(static_cast<uint8_t>(comms.channel_state(ChannelId::Wifi)));
                    payload.push_back(static_cast<uint8_t>(comms.channel_state(ChannelId::Radio)));

                    uint32_t uptime = static_cast<uint32_t>(
                        std::chrono::duration_cast<std::chrono::seconds>(
                            std::chrono::steady_clock::now() - process_start
                        ).count()
                    );
                    appendBytes(payload, uptime);

                    CommsMessage hb;
                    hb.type = MessageType::I_BRD;
                    hb.src = sat_src;
                    hb.dest = 0x01;
                    hb.correlation_id = 0;
                    hb.payload = payload;

                    hb.channel_hint = ChannelId::Wifi;
                    comms.send(hb);
                    hb.channel_hint = ChannelId::Radio;
                    comms.send(hb);

                    break;
                }
                default: break;
            }
        }
        catch(const QecException& ex){
            std::cerr << "[ERROR][MAIN] seq=" << e.seq
                << " type=" << static_cast<int>(e.type)
                << " " << ex.what() << '\n';
        }
        catch(const std::exception& ex){
            std::cerr << "[ERROR][MAIN] seq=" << e.seq
                << " type=" << static_cast<int>(e.type)
                << " std::exception: " << ex.what() << '\n';
        }
        catch(...){
            std::cerr << "[ERROR][MAIN] seq=" << e.seq
                << " type=" << static_cast<int>(e.type)
                << " unknown exception\n";
        }
    }

    // (Unreached)
    deploy.stop();
    cams.shutdown();
    imu.stop();
    comms.stop();
    return 0;
}

int main(){
    try{
        return run();
    }
    catch(const QecException& ex){
        std::cerr << "[FATAL] " << ex.what() << '\n';
        return 1;
    }
    catch(const std::exception& ex){
        std::cerr << "[FATAL] std::exception: " << ex.what() << '\n';
        return 1;
    }
    catch (...){
        std::cerr << "[FATAL] unknown exception\n";
        return 1;
    }
}
