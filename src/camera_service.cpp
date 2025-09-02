// src/camera_service.cpp
#include "camera_service.hpp"   // declares CameraModule + CameraModuleConfig
#include "../include/events.hpp"           // Event, EventType, EvPhotoTaken etc
#include "../include/tsqueue.hpp"          // TSQueue<Event>

#include <libcamera/libcamera.h>
#include <libcamera/camera_manager.h>
#include <libcamera/framebuffer_allocator.h>
#include <libcamera/request.h>
#include <libcamera/stream.h>

#include <jpeglib.h>

#include <atomic>
#include <condition_variable>
#include <cstdio>
#include <functional>
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
// tbh i dont know why i didnt use the tsqueue but we move
template <typename T>
class CmdQueue {
public:
    void push(const T& v) { { std::lock_guard<std::mutex> lk(m_); q_.push_back(v); } cv_.notify_one(); }
    T pop() {
        std::unique_lock<std::mutex> lk(m_);
        cv_.wait(lk, [&]{ return !q_.empty(); });
        T v = std::move(q_.front()); q_.erase(q_.begin()); return v;
    }
private:
    std::vector<T> q_;
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

    // init this camera (alloc, map, start)
    bool init(TSQueue<Event>& out, int index, unsigned w, unsigned h, int warmup, int quality) {
        outQ = &out; cam_index = index; width = w; height = h; warmup_frames = warmup; jpeg_quality = quality;

        auto& cm = get_cm();
        const auto& cams = cm.cameras();
        if (cam_index < 0 || cam_index >= static_cast<int>(cams.size())) {
            std::fprintf(stderr, "[Cam%d] invalid index (have %zu)\n", cam_index, cams.size());
            return false;
        }

        cam = cams[cam_index];
        allocator = std::make_unique<FrameBufferAllocator>(cam);

        if (cam->acquire()) { std::fprintf(stderr, "[Cam%d] acquire failed\n", cam_index); return false; }

        auto conf = cam->generateConfiguration({ StreamRole::StillCapture });
        if (!conf) { std::fprintf(stderr, "[Cam%d] gen config failed\n", cam_index); return false; }
        conf->at(0).pixelFormat = formats::RGB888; // we do BGR->RGB swap in saveJPEG
        conf->at(0).size = { width, height };
        (void)conf->validate();
        if (cam->configure(conf.get()) < 0) { std::fprintf(stderr, "[Cam%d] configure failed\n", cam_index); return false; }

        stream = conf->at(0).stream();
        if (allocator->allocate(stream) < 0) { std::fprintf(stderr, "[Cam%d] alloc failed\n", cam_index); return false; }
        auto& bufs = allocator->buffers(stream);
        if (bufs.empty()) { std::fprintf(stderr, "[Cam%d] no buffers\n", cam_index); return false; }

        // map plane 0 of buffer 0
        const auto& pl = bufs[0]->planes()[0];
        void* addr = mmap(nullptr, pl.length, PROT_READ, MAP_SHARED, pl.fd.get(), 0);
        if (addr == MAP_FAILED) { std::perror("mmap"); return false; }
        mapped_addr = static_cast<uint8_t*>(addr);
        mapped_len  = pl.length;

        // persistent request
        req = cam->createRequest();
        if (!req) { std::fprintf(stderr, "[Cam%d] createRequest failed\n", cam_index); return false; }
        if (req->addBuffer(stream, bufs[0].get()) < 0) { std::fprintf(stderr, "[Cam%d] addBuffer failed\n", cam_index); return false; }

        cam->requestCompleted.connect(this, &OneCam::onRequestComplete);

        if (cam->start() < 0) { std::fprintf(stderr, "[Cam%d] start failed\n", cam_index); return false; }

        running = true;
        worker = std::thread([this]{ run(); });
        return true;
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
    void run() {
        while (true) {
            auto cmd = inbox.pop();
            if (std::holds_alternative<CmdStop>(cmd)) break;

            const auto& tk = std::get<CmdTake>(cmd);
            pending_path = tk.path; pending_seq = tk.seq;

            // warm-up
            for (int i = 0; i < warmup_frames; ++i) {
                if (!queueAndWaitOne()) { pushPhotoTaken(false, "warmup failed"); goto next; }
            }
            // real capture
            if (!queueAndWaitOne()) { pushPhotoTaken(false, "capture failed"); goto next; }
            // save
            if (!saveJPEG(pending_path)) { pushPhotoTaken(false, "jpeg failed"); }
            else                         { pushPhotoTaken(true,  ""); }

        next:
            continue;
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

    bool queueAndWaitOne() {
        {
            std::lock_guard<std::mutex> lk(done_mx);
            frame_done = false;
            last_ok = false;
        }
        req->reuse(Request::ReuseBuffers);
        if (cam->queueRequest(req.get()) < 0) return false;

        std::unique_lock<std::mutex> lk(done_mx);
        done_cv.wait(lk, [&]{ return frame_done; });
        return last_ok;
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
        Event e;
        e.type = EventType::PhotoTaken;
        // Fill your payload shape here. Minimal example uses just a path:
        e.data = EvPhotoTaken{ pending_path };
        // If EvPhotoTaken has more fields, use them:
        // e.data = EvPhotoTaken{ pending_path, pending_seq, ok, cam_index, err };
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
        left_ok  = left .init(outQ, cfg.left_index,  cfg.width, cfg.height, cfg.warmup_frames, cfg.jpeg_quality);
        right_ok = right.init(outQ, cfg.right_index, cfg.width, cfg.height, cfg.warmup_frames, cfg.jpeg_quality);
        return left_ok && right_ok;
    }
    void stop_both() {
        right.stop();
        left .stop();
    }
};

CameraModule::CameraModule(TSQueue<Event>& mainEventQueue) : d_(new Impl(mainEventQueue)) {}
CameraModule::~CameraModule() { shutdown(); }

bool CameraModule::startup(const CameraModuleConfig& cfg) {
    return d_->start_both(cfg);
}

void CameraModule::take_left(const std::string& path, uint32_t seq)  { d_->left.take(path,  seq); }
void CameraModule::take_right(const std::string& path, uint32_t seq) { d_->right.take(path, seq); }

void CameraModule::take_both(const std::string& left_path, const std::string& right_path, uint32_t seq) {
    // Software near-simultaneous: post left then right quickly.
    d_->left.take (left_path,  seq);
    d_->right.take(right_path, seq);
}

void CameraModule::shutdown() {
    if (!d_) return;
    d_->stop_both();
}
