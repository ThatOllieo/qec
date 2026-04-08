#pragma once

#include <stdexcept>
#include <vector>
#include <string>
#include <utility>
#include <sstream>

enum class ErrorCode{
    Unknown,
    InvalidArgument,
    ConfigError,
    StartupFailed,
    IOError,
    Timeout,
    DeviceUnreachable,
    ProtocolError,
    StateError,
    ExternalCommandFailed
};

enum class ErrorSeverity{
    Warning,
    Recoverable,
    Fatal
};

class QecException : public std::runtime_error{
public:
    using Context = std::vector<std::pair<std::string,std::string>>;

    QecException(
        ErrorCode code,
        ErrorSeverity severity,
        std::string message,
        std::string location,
        Context context = {},
        std::string cause = {}
    )
    : std::runtime_error(buildWhat(message, location, code, severity, context, cause)),
        code_(code),
        severity_(severity),
        message_(std::move(message)),
        location_(std::move(location)),
        context_(std::move(context)),
        cause_(std::move(cause)) {}

    ErrorCode code() const noexcept {return code_;}
    ErrorSeverity severity() const noexcept {return severity_;}
    const std::string& message() const noexcept {return message_;}
    const std::string& location() const noexcept {return location_;}
    const Context& context() const noexcept {return context_;}
    const std::string& cause() const noexcept {return cause_;}

private:
    static std::string buildWhat(
        const std::string& message,
        const std::string& location,
        ErrorCode code,
        ErrorSeverity severity,
        const Context& context,
        const std::string& cause
    ){
        std::ostringstream oss;
        oss << message;
        if(!location.empty()){
            oss << " | location=" << location;
        }
        oss << " | code=" << static_cast<int>(code);
        oss << " | severity=" << static_cast<int>(severity);

        for(const auto& [k,v] : context){
            oss << " | " << k << "=" << v;
        }
        if(!cause.empty()){
            oss << " | cause=" << cause;
        }

        return oss.str();
    }

    ErrorCode code_;
    ErrorSeverity severity_;
    std::string message_;
    std::string location_;
    Context context_;
    std::string cause_;
};

class CommsError : public QecException{ public: using QecException:QecException; };
class CamsError : public QecException{ public: using QecException:QecException; };
class DeployWatchError : public QecException{ public: using QecException:QecException; };
class IMUError : public QecException{ public: using QecException:QecException; };