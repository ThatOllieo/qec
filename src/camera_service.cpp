// src/camera_service.cpp
#include "camera_service.hpp"   // declares CameraModule + CameraModuleConfig
#include "../include/events.hpp"           // Event, EventType, EvPhotoTaken etc
#include "../include/tsqueue.hpp"          // TSQueue<Event>
#include "../include/logger.hpp"

#include <libcamera/libcamera.h>
#include <libcamera/camera_manager.h>
#include <libcamera/framebuffer_allocator.h>
#include <libcamera/request.h>
#include <libcamera/stream.h>

#include <jpeglib.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <deque>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <variant>
#include <vector>
#include <sys/mman.h>

using namespace libcamera;

// ================== Global CameraManager (hidden) ==================
static CameraManager& get_cm() {
    static CameraManager cm;
    static bool started = false;
    if (!started) {
        if (cm.start()) throw std::runtime_error("CameraManager start failed");
        started = true;
    }
    return cm;
}

// ================== Tiny command queue (internal) ==================
template <typename T>
class CmdQueue {
public:
    void push(const T& v) { { std::lock_guard<std::mutex> lk(m_); q_.push_back(v); } cv_.notify_one(); }
    T pop() {
        std::unique_lock<std::mutex> lk(m_);
        cv_.wait(lk, [&]{ return !q_.empty(); });
        T v = std::move(q_.front()); q_.pop_front(); return v;
    }
private:
    std::deque<T> q_;
    std::mutex m_;
    std::condition_variable cv_;
};

// ================== One camera worker (internal) ==================
struct OneCam {
    // config
    int cam_index = -1;
    unsigned width = 1920, height = 1080;
    int warmup_frames = 8;
    int jpeg_quality  = 85;
    std::string worker_name = "camera_worker";

    // output to main
    TSQueue<Event>* outQ = nullptr;

    // libcamera
    std::shared_ptr<Camera> cam;
    std::unique_ptr<FrameBufferAllocator> allocator; // non-copyable, so use unique_ptr
    Stream* stream = nullptr;
    std::unique_ptr<Request> req;                    // persistent request

    // mapped plane 0
    uint8_t* mapped_addr = nullptr;
    size_t   mapped_len  = 0;

    // worker thread + commands
    struct CmdTake { std::string path; uint32_t seq; };
    struct CmdStop {};
    CmdQueue<std::variant<CmdTake, CmdStop>> inbox;
    std::thread worker;
    std::atomic<bool> running{false};

    // completion sync (callback -> worker)
    std::mutex done_mx;
    std::condition_variable done_cv;
    bool frame_done = false;
    bool last_ok    = false;

    // current shot
    std::string pending_path;
    uint32_t    pending_seq = 0;

    // error captured by init() on failure, for CameraModule::startup() to report
    std::string init_error_;

    enum class CaptureResult { Ok, Failed, TimedOut };

    // init this camera (alloc, map, start)
    bool init(TSQueue<Event>& out, int index, unsigned w, unsigned h, int warmup, int quality, const std::string& name) {
        outQ = &out; cam_index = index; width = w; height = h; warmup_frames = warmup; jpeg_quality = quality;
        worker_name = name;

        bool did_acquire  = false;
        bool did_allocate = false;
        bool did_map      = false;

        try {
            auto& cm = get_cm();
            const auto& cams = cm.cameras();
            if (cam_index < 0 || cam_index >= static_cast<int>(cams.size())) {
                throw CamsError(ErrorCode::StartupFailed, ErrorSeverity::Recoverable,
                    "Invalid camera index", "OneCam::init",
                    {{"cam_index", std::to_string(cam_index)}, {"step", "index_check"}, {"available", std::to_string(cams.size())}});
            }

            cam = cams[cam_index];
            allocator = std::make_unique<FrameBufferAllocator>(cam);

            if (cam->acquire()) {
                throw CamsError(ErrorCode::StartupFailed, ErrorSeverity::Recoverable,
                    "Camera acquire failed", "OneCam::init",
                    {{"cam_index", std::to_string(cam_index)}, {"step", "acquire"}});
            }
            did_acquire = true;

            auto conf = cam->generateConfiguration({ StreamRole::StillCapture });
            if (!conf) {
                throw CamsError(ErrorCode::StartupFailed, ErrorSeverity::Recoverable,
                    "generateConfiguration failed", "OneCam::init",
                    {{"cam_index", std::to_string(cam_index)}, {"step", "generateConfiguration"}});
            }
            conf->at(0).pixelFormat = formats::RGB888; // we do BGR->RGB swap in saveJPEG
            conf->at(0).size = { width, height };
            (void)conf->validate();
            if (cam->configure(conf.get()) < 0) {
                throw CamsError(ErrorCode::StartupFailed, ErrorSeverity::Recoverable,
                    "configure failed", "OneCam::init",
                    {{"cam_index", std::to_string(cam_index)}, {"step", "configure"}});
            }

            stream = conf->at(0).stream();
            if (allocator->allocate(stream) < 0) {
                throw CamsError(ErrorCode::StartupFailed, ErrorSeverity::Recoverable,
                    "buffer allocate failed", "OneCam::init",
                    {{"cam_index", std::to_string(cam_index)}, {"step", "allocate"}});
            }
            did_allocate = true;
            auto& bufs = allocator->buffers(stream);
            if (bufs.empty()) {
                throw CamsError(ErrorCode::StartupFailed, ErrorSeverity::Recoverable,
                    "no buffers allocated", "OneCam::init",
                    {{"cam_index", std::to_string(cam_index)}, {"step", "allocate"}});
            }

            // map plane 0 of buffer 0
            const auto& pl = bufs[0]->planes()[0];
            void* addr = mmap(nullptr, pl.length, PROT_READ, MAP_SHARED, pl.fd.get(), 0);
            if (addr == MAP_FAILED) {
                throw CamsError(ErrorCode::IOError, ErrorSeverity::Recoverable,
                    "mmap failed", "OneCam::init",
                    {{"cam_index", std::to_string(cam_index)}, {"step", "mmap"}}, std::strerror(errno));
            }
            mapped_addr = static_cast<uint8_t*>(addr);
            mapped_len  = pl.length;
            did_map = true;

            // persistent request
            req = cam->createRequest();
            if (!req) {
                throw CamsError(ErrorCode::StartupFailed, ErrorSeverity::Recoverable,
                    "createRequest failed", "OneCam::init",
                    {{"cam_index", std::to_string(cam_index)}, {"step", "createRequest"}});
            }
            if (req->addBuffer(stream, bufs[0].get()) < 0) {
                throw CamsError(ErrorCode::StartupFailed, ErrorSeverity::Recoverable,
                    "addBuffer failed", "OneCam::init",
                    {{"cam_index", std::to_string(cam_index)}, {"step", "addBuffer"}});
            }

            cam->requestCompleted.connect(this, &OneCam::onRequestComplete);

            if (cam->start() < 0) {
                throw CamsError(ErrorCode::StartupFailed, ErrorSeverity::Recoverable,
                    "camera start failed", "OneCam::init",
                    {{"cam_index", std::to_string(cam_index)}, {"step", "start"}});
            }

            running = true;
            worker = std::thread([this]{ run(); });
            return true;
        } catch (const CamsError& ex) {
            init_error_ = ex.what();
            logLine(ex.severity(), worker_name, init_error_);

            // Unwind only the steps that actually completed, symmetric to stop().
            if (did_map)      { munmap(mapped_addr, mapped_len); mapped_addr = nullptr; mapped_len = 0; }
            if (did_allocate && stream && allocator) allocator->free(stream);
            if (did_acquire && cam) cam->release();
            return false;
        }
    }

    void stop() {
        // tell worker to exit
        inbox.push(CmdStop{});
        if (worker.joinable()) worker.join();

        if (running.exchange(false)) {
            cam->stop();
            if (mapped_addr) { munmap(mapped_addr, mapped_len); mapped_addr = nullptr; mapped_len = 0; }
            if (stream && allocator) allocator->free(stream);
            cam->release();
        }
    }

    void take(const std::string& path, uint32_t seq) {
        inbox.push(CmdTake{ path, seq });
    }

    // ===== internals =====
    void handleTake(const CmdTake& tk) {
        pending_path = tk.path; pending_seq = tk.seq;

        // warm-up
        for (int i = 0; i < warmup_frames; ++i) {
            auto res = queueAndWaitOne();
            if (res != CaptureResult::Ok) {
                pushPhotoTaken(false, res == CaptureResult::TimedOut ? "warmup timed out" : "warmup failed");
                return;
            }
        }
        // real capture
        auto res = queueAndWaitOne();
        if (res != CaptureResult::Ok) {
            pushPhotoTaken(false, res == CaptureResult::TimedOut ? "capture timed out" : "capture failed");
            return;
        }
        // save
        if (!saveJPEG(pending_path)) { pushPhotoTaken(false, "jpeg failed"); }
        else                         { pushPhotoTaken(true,  ""); }
    }

    void run() {
        while (true) {
            try {
                auto cmd = inbox.pop();
                if (std::holds_alternative<CmdStop>(cmd)) break;

                handleTake(std::get<CmdTake>(cmd));
            } catch (const std::exception& ex) {
                logLine(ErrorSeverity::Recoverable, worker_name, std::string("worker loop error: ") + ex.what());
                if (outQ) {
                    Event e; e.type = EventType::ModuleFailed;
                    e.data = EvModuleFailed{worker_name, ex.what(), ErrorSeverity::Recoverable};
                    outQ->push(std::move(e));
                }
            } catch (...) {
                logLine(ErrorSeverity::Recoverable, worker_name, "worker loop unknown error");
                if (outQ) {
                    Event e; e.type = EventType::ModuleFailed;
                    e.data = EvModuleFailed{worker_name, "unknown exception", ErrorSeverity::Recoverable};
                    outQ->push(std::move(e));
                }
            }
        }
    }

    void onRequestComplete(Request* r) {
        bool ok = (r->status() == Request::RequestComplete);
        {
            std::lock_guard<std::mutex> lk(done_mx);
            frame_done = true;
            last_ok = ok;
        }
        done_cv.notify_one();
    }

    CaptureResult queueAndWaitOne() {
        {
            std::lock_guard<std::mutex> lk(done_mx);
            frame_done = false;
            last_ok = false;
        }
        req->reuse(Request::ReuseBuffers);
        if (cam->queueRequest(req.get()) < 0) return CaptureResult::Failed;

        std::unique_lock<std::mutex> lk(done_mx);
        bool got = done_cv.wait_for(lk, std::chrono::seconds(5), [&]{ return frame_done; });
        if (!got) return CaptureResult::TimedOut;
        return last_ok ? CaptureResult::Ok : CaptureResult::Failed;
    }

    bool saveJPEG(const std::string& out) {
        std::cout << "saving jpeg" << std::endl;
        if (!mapped_addr) return false;
        const int w = static_cast<int>(width);
        const int h = static_cast<int>(height);
        const size_t need = static_cast<size_t>(w) * h * 3;
        if (mapped_len < need) { std::fprintf(stderr, "[Cam%d] buffer too small\n", cam_index); return false; }

        // BGR -> RGB swap (common with RGB888 on Pi)
        std::vector<uint8_t> rgb(need);
        for (int i = 0; i < w*h; ++i) {
            rgb[i*3+0] = mapped_addr[i*3+2];
            rgb[i*3+1] = mapped_addr[i*3+1];
            rgb[i*3+2] = mapped_addr[i*3+0];
        }

        jpeg_compress_struct cinfo{}; jpeg_error_mgr jerr{};
        cinfo.err = jpeg_std_error(&jerr);
        jpeg_create_compress(&cinfo);
        FILE* f = std::fopen(out.c_str(), "wb");
        if (!f) { jpeg_destroy_compress(&cinfo); return false; }
        jpeg_stdio_dest(&cinfo, f);
        cinfo.image_width = w;
        cinfo.image_height = h;
        cinfo.input_components = 3;
        cinfo.in_color_space = JCS_RGB;
        jpeg_set_defaults(&cinfo);
        jpeg_set_quality(&cinfo, jpeg_quality, TRUE);
        jpeg_start_compress(&cinfo, TRUE);

        JSAMPROW row[1];
        while (cinfo.next_scanline < cinfo.image_height) {
            row[0] = &rgb[cinfo.next_scanline * w * 3];
            jpeg_write_scanlines(&cinfo, row, 1);
        }
        jpeg_finish_compress(&cinfo);
        std::fclose(f);
        jpeg_destroy_compress(&cinfo);
        return true;
    }

    void pushPhotoTaken(bool ok, const std::string& err) {
        if (!ok) {
            logLine(ErrorSeverity::Warning, worker_name, "Capture failed: " + err);
        }
        Event e;
        e.type = EventType::PhotoTaken;
        e.data = EvPhotoTaken{ pending_path, ok, err };
        outQ->push(std::move(e));
    }
};

// ================== CameraModule implementation ==================
struct CameraModule::Impl {
    TSQueue<Event>& outQ;
    CameraModuleConfig cfg{};
    OneCam left;
    OneCam right;
    bool left_ok  = false;
    bool right_ok = false;

    explicit Impl(TSQueue<Event>& q) : outQ(q) {}

    bool start_both(const CameraModuleConfig& c) {
        cfg = c;
        left_ok  = left .init(outQ, cfg.left_index,  cfg.width, cfg.height, cfg.warmup_frames, cfg.jpeg_quality, "camera_worker_left");
        right_ok = right.init(outQ, cfg.right_index, cfg.width, cfg.height, cfg.warmup_frames, cfg.jpeg_quality, "camera_worker_right");
        return left_ok && right_ok;
    }

    std::string combinedError() const {
        std::string msg;
        if (!left_ok)  { msg += "left=" + left.init_error_; }
        if (!right_ok) { if (!msg.empty()) msg += ", "; msg += "right=" + right.init_error_; }
        return msg;
    }

    bool stop_both() {
        right.stop();
        left .stop();
        return true;
    }
};

CameraModule::CameraModule(TSQueue<Event>& mainEventQueue) : d_(new Impl(mainEventQueue)) {}
CameraModule::~CameraModule() { shutdown(); }

void CameraModule::startup(const CameraModuleConfig& cfg) {
    state_ = ModuleState::Starting;
    if(d_->start_both(cfg)){
        state_ = ModuleState::Running;
    }
    else{
        throw CamsError(
            ErrorCode::StartupFailed,
            ErrorSeverity::Recoverable,
            "CameraModule failed to start: " + d_->combinedError(),
            "CameraModule::startup"
        );
    }
}

void CameraModule::take_left(const std::string& path, uint32_t seq)  { d_->left.take(path,  seq); }
void CameraModule::take_right(const std::string& path, uint32_t seq) { d_->right.take(path, seq); }

void CameraModule::take_both(const std::string& left_path, const std::string& right_path, uint32_t seq) {
    // Software near-simultaneous: post left then right quickly.
    if(state_ == ModuleState::Running){
        d_->left.take (left_path,  seq);
        d_->right.take(right_path, seq);
    }
    else{
        throw CamsError(
            ErrorCode::StateError,
            ErrorSeverity::Warning,
            "Attempted to take photos before cameras had started.",
            "CameraModule::take_both"
        );

    }
}

void CameraModule::shutdown() {
    if (!d_) return;
    if(d_->stop_both()){
        state_ = ModuleState::Stopped;
    }
}
