#include "../include/ws_link.hpp"

#include <atomic>
#include <thread>
#include <string>
#include <memory>

#include <boost/asio.hpp>
#include <boost/beast.hpp>
#include <boost/beast/websocket.hpp>

#include <iostream>
#include <deque>
#include <unordered_set>
#include <mutex>

namespace asio = boost::asio;
namespace beast = boost::beast;
namespace http = beast::http;
namespace ws = beast::websocket;
using tcp = asio::ip::tcp;


struct WSLink::Impl {
    TSQueue<Event>& q_;

    asio::io_context ioc_;
    std::unique_ptr<tcp::acceptor> acceptor_;
    std::thread th_;
    std::atomic<bool> running_{false};

    struct Session;

    std::mutex sessions_mtx_;
    std::unordered_set<std::shared_ptr<Session>> sessions_;

    struct Session : std::enable_shared_from_this<Session> {
        ws::stream<tcp::socket> ws_;
        beast::flat_buffer readBuf_;

        std::deque<std::string> outQ_;
        bool writing_ = false;

        Impl& server_;

        explicit Session(tcp::socket sock, Impl& server)
            : ws_(std::move(sock)), server_(server) {}

        void start() {
            ws_.set_option(ws::stream_base::timeout::suggested(beast::role_type::server));
            ws_.set_option(ws::stream_base::decorator([](ws::response_type& res) {
                res.set(http::field::server, "juk-gs");
            }));
            ws_.async_accept(beast::bind_front_handler(&Session::on_accept, shared_from_this()));
        }

        void on_accept(beast::error_code ec) {
            if (ec) return fail("accept", ec);
            do_read();
        }

        void do_read() {
            ws_.async_read(readBuf_, beast::bind_front_handler(&Session::on_read, shared_from_this()));
        }

        void on_read(beast::error_code ec, std::size_t) {
            if (ec == ws::error::closed) {
                server_.remove_session(shared_from_this());
                return;
            }
            if (ec) {
                fail("read", ec);
                server_.remove_session(shared_from_this());
                return;
            }

            std::string msg = beast::buffers_to_string(readBuf_.data());
            readBuf_.consume(readBuf_.size());

            server_.push_incoming_message(std::move(msg));

            do_read();
        }

        void send(std::string text) {
            outQ_.push_back(std::move(text));
            if (!writing_) {
                writing_ = true;
                do_write();
            }
        }

        void do_write() {
            ws_.text(true);
            ws_.async_write(
                asio::buffer(outQ_.front()),
                beast::bind_front_handler(&Session::on_write, shared_from_this())
            );
        }

        void on_write(beast::error_code ec, std::size_t) {
            if (ec) {
                fail("write", ec);
                server_.remove_session(shared_from_this());
                return;
            }

            outQ_.pop_front();
            if (!outQ_.empty()) {
                do_write();
            } else {
                writing_ = false;
            }
        }

        static void fail(const char* what, beast::error_code ec) {
            std::cerr << "[ws session] " << what << ": " << ec.message() << "\n";
        }
    };

    explicit Impl(TSQueue<Event>& q) : q_(q) {}

    ~Impl() { stop(); }

    void start(uint16_t port) {
        if (running_) return;
        running_ = true;

        acceptor_ = std::make_unique<tcp::acceptor>(ioc_, tcp::endpoint(tcp::v4(), port));
        do_accept();

        th_ = std::thread([this] {
            try {
                ioc_.run();
            } catch (const std::exception& e) {
                std::cerr << "[ws server] io_context exception: " << e.what() << "\n";
            }
        });

        std::cout << "[ws server] listening on ws://<pi>:" << port << "/ws\n";
    }

    void stop() {
        if (!running_) return;
        running_ = false;

        asio::post(ioc_, [this] {
            beast::error_code ec;

            if (acceptor_) {
                acceptor_->close(ec);
                acceptor_.reset();
            }

            std::lock_guard<std::mutex> lock(sessions_mtx_);
            for (auto& s : sessions_) {
                if (!s) continue;
                s->ws_.close(ws::close_code::normal, ec);
            }
            sessions_.clear();
        });

        ioc_.stop();
        if (th_.joinable()) th_.join();
    }

    void do_accept() {
        if (!acceptor_) return;

        acceptor_->async_accept([this](beast::error_code ec, tcp::socket socket) {
            if (!acceptor_) return; 

            if (!ec) {
                auto s = std::make_shared<Session>(std::move(socket), *this);
                {
                    std::lock_guard<std::mutex> lock(sessions_mtx_);
                    sessions_.insert(s);
                }
                s->start();
            } else {
                std::cerr << "[ws server] accept: " << ec.message() << "\n";
            }

            if (running_) do_accept();
        });
    }

    void broadcast(std::string jsonText) {
        asio::post(ioc_, [this, text = std::move(jsonText)]() mutable {
            std::lock_guard<std::mutex> lock(sessions_mtx_);
            for (auto& s : sessions_) {
                if (s) s->send(text);
            }
        });
    }

    void remove_session(const std::shared_ptr<Session>& s) {
        asio::post(ioc_, [this, s] {
            std::lock_guard<std::mutex> lock(sessions_mtx_);
            sessions_.erase(s);
        });
    }

    void push_incoming_message(std::string msg) {
        std::cout << "[ws <- client] " << msg << "\n";
    }
};


WSLink::WSLink(TSQueue<Event>& mainQueue)
    : impl_(std::make_unique<Impl>(mainQueue)) {}

WSLink::~WSLink() = default;

void WSLink::start(uint16_t port) { impl_->start(port); }
void WSLink::stop() { impl_->stop(); }
void WSLink::broadcast(std::string jsonText) { impl_->broadcast(std::move(jsonText)); }