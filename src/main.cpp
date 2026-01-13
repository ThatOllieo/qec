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

#include <fstream>
#include <string>

//Quick demo function to get the temperature of the cm5's cpu
float getCPUTemperature() {
    std::ifstream file("/sys/class/thermal/thermal_zone0/temp");
    if (!file.is_open()) {
        std::cerr << "Error: Unable to open temperature file!" << std::endl;
        return -1.0f;
    }

    std::string tempStr;
    std::getline(file, tempStr);
    file.close();

    // Convert the temperature to degrees Celsius
    return std::stof(tempStr) / 1000.0f;
}

// Pending async plans for tracking replies (when this file makes requests and sends commands)
static std::unordered_map<uint16_t, std::function<void(const std::vector<uint8_t>&)>> telem_plans;
static std::unordered_map<uint16_t, std::function<void()>> cmd_plans;

//function called that gets the data for each sensor id
static std::vector<uint8_t> gather_telem_bytes(uint16_t sensor_id, DeploymentWatcher& deploy, IMU& imu) {
    switch (sensor_id)
    {
    //deploy switch
    case 0x0001:
        return {static_cast<uint8_t>(deploy.getState())};
        break;
    //cpu temp
    case 0x0003:
        return {getCPUTemperature()};
        break;
    
    //imu calibration status
    case 0x2010: {
        IMU::CalibrationStatus calibstat = imu.getCalibrationStatus();
        return {calibstat.sys,calibstat.gyro,calibstat.accel,calibstat.mag}; }
        break;
    //imu euler angles
    case 0x2020: {
        std::vector<uint8_t> pkt;
        IMU::EulerAngles eulang = imu.getEulerAngles();
        pkt.reserve(64);
        appendBytes(pkt, eulang.heading);
        appendBytes(pkt, eulang.roll);
        appendBytes(pkt, eulang.pitch);
        return {pkt};
        break;
    }
        

    //imu quaternion
    case 0x2030: {
        std::vector<uint8_t> pkt;
        IMU::Quaternion quat = imu.getQuaternion();
        pkt.reserve(64);
        appendBytes(pkt, quat.w);
        appendBytes(pkt, quat.x);
        appendBytes(pkt, quat.y);
        appendBytes(pkt, quat.z);
        return {pkt};
        break;
    }
        

    //imu gravity vector
    case 0x2040: {
        std::vector<uint8_t> pkt;
        IMU::Vector3 grav = imu.getGravity();
        pkt.reserve(64);
        appendBytes(pkt, grav.x);
        appendBytes(pkt, grav.y);
        appendBytes(pkt, grav.z);
        return {pkt};
        break;
    }

    //imu linear accel.
    case 0x2050:{
        std::vector<uint8_t> pkt;
        IMU::Vector3 linacc = imu.getLinearAccel();
        pkt.reserve(64);
        appendBytes(pkt, linacc.x);
        appendBytes(pkt, linacc.y);
        appendBytes(pkt, linacc.z);
        return {pkt};
        break;
    }

    //fallback (deadbeef). Just a default, can be used as a detection of no sensor found or just to test the connection is working.
    default:
        return { uint8_t(sensor_id & 0xFF), uint8_t(sensor_id >> 8), 0xDE, 0xAD, 0xBE, 0xEF };
        break;
    }
    
}

//handler for incoming commands
int handleCommand(auto& c, CommsManager& comms, CameraModule& cams){
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
    //reboot
    case 0x02:
        comms.reply_ok_command(c.correlation_id);
        system("sudo reboot");

    //manual image capture trigger
    case 0x11:
        cams.take_both("/home/pi/left.jpg","/home/pi/right.jpg", 42);
        comms.reply_ok_command(c.correlation_id); 
        break;

    //send saved images
    case 0x12:     
        setTimeout([&c, &comms]() {
            system("scp /home/pi/left.jpg /home/pi/right.jpg pi@10.42.0.1:/var/www/juk/media/");
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

    //COMMS MANAGER CONTROLS 0x1***
    //restart WiFi channel
    case 0x1011:
        setTimeout([&c, &comms]() {
            comms.restart_channel(ChannelId::Wifi);
            comms.reply_ok_command(c.correlation_id);
        }, std::chrono::milliseconds(200));
        break;

    //start WiFi channel
    case 0x1012:
        setTimeout([&c, &comms]() {
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
        setTimeout([&c, &comms]() {
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

    return 0;
}


int main() {
    // create main event list, this is the core part of this software, everything that happens is submitted as an event to this list
    // for the main thread to act upon
    TSQueue<Event> eventList;

    //camera setup, note that the events list is passed by reference so that events can be added by the camera thread. e.g. ive taken a photo
    CameraModule cams(eventList);

    CameraModuleConfig cfg;
    cfg.left_index = 0;
    cfg.right_index = 1;
    cfg.width = 3840;
    cfg.height = 2160;
    cfg.warmup_frames = 8;
    cfg.jpeg_quality = 85;

    if (!cams.startup(cfg)) {
        std::cerr << "Camera startup failed\n";
        //return 1;
    }

    //Deployment switch, configured so that on release of the switch, an event is pushed to the main list
    DeploymentWatcher deploy(eventList);
    deploy.start();

    //Imu is started, required for the IMU telemetry requests or other onboard use
    IMU imu(eventList);
    imu.start();

    //Comms manager is what handles all off board communications, incoming messages become events in the main list, and main thread can
    // simply call functions to request data or send commands elsewhere.
    CommsManager comms(eventList);
    comms.set_src(0xEF); // sets the identifier of this device

    //TcpConfig tcpcfg;
    //tcpcfg.host = "10.42.0.1";  
    //tcpcfg.port = 5000;
    //auto tcp = std::make_unique<TcpChannel>(tcpcfg);
    //comms.register_channel(std::move(tcp));

    UdpConfig udpcfg;
    auto udp =  std::make_unique<UdpChannel>(udpcfg);
    comms.register_channel(std::move(udp));

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

    comms.start();

    // --- DEMO outbound requests: fire once on startup ---
    {
        // Telemetry request demo (sensor 7)
        uint16_t corr = comms.request_telem_async(
            /*dest*/0x01,
            ChannelId::Wifi,
            /*sensor*/7,
            std::chrono::milliseconds(1000),
            /*retries*/2
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

    uint64_t seq = 1; //counter, used for logging 
    for (;;) {
        Event e = eventList.pop(); // blocks until an event arrives
        e.seq = seq++;

        switch (e.type) {
            //on satellite deployment switch trigger, cameras are told to take photos 
            case EventType::DeploymentTriggered: {
                const auto& d = std::get<EvDeploymentTriggered>(e.data);
                std::cout << "[MAIN] DEPLOY" << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                cams.take_both("/home/pi/left.jpg","/home/pi/right.jpg", 42);
                imu.triggerCapture(10000);
                break;
            }
            
            case EventType::MotionCaptured:{
                const auto& m = std::get<EvMotionCaptured>(e.data);
                std::string path = m.path;
                setTimeout([path,&comms](){
                    std::cout << "[MAIN] Got mocap event, for path " << path << std::endl;
                    system((std::string("scp ") + path + " pi@10.42.0.1:/home/pi/startup.csv").c_str());
                }, std::chrono::milliseconds(1));
                break;
            }

            //on photo taken, these are automatically sent to the groundstation address
            case EventType::PhotoTaken: {
                const auto& p = std::get<EvPhotoTaken>(e.data);
                std::cout << "[MAIN] (seq " << e.seq << ") PhotoTaken: " << p.path << "\n";

                setTimeout([&p, &comms]() {
                    std::cout << "[MAIN] triggering scp send\n";
                    system("scp /home/pi/left.jpg /home/pi/right.jpg pi@10.42.0.1:/var/www/juk/media/");
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
                handleCommand(c, comms, cams);
                break;

            }
            //on telemetry request, get telem for specifed sensor, reply
            case EventType::TelemetryRequest: {
                auto& t = std::get<EvTelemetryRequest>(e.data);
                std::cout << "[MAIN] Tlm CorrId: " << t.correlation_id << std::endl;
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
            default: break;
        }
    }

    // (Unreached)
    deploy.stop();
    cams.shutdown();
    imu.stop();
    comms.stop();
    return 0;
}
