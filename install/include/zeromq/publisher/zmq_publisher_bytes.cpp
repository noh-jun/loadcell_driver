#include "zmq_publisher_bytes.h"

#include <cerrno>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <utility>

#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

namespace zmq_pub {
namespace {

constexpr int kLingerMillis = 0;
constexpr int kSendHighWaterMark = 3;
constexpr int kSendTimeoutMillis = 10;

pid_t CurrentPid() {
  return ::getpid();
}

std::string ErrnoToString(int error_no) {
  return std::string(::strerror(error_no));
}

void ValidateNonEmptyOrThrow(const std::string& value,
                            std::string_view field_name,
                            std::string_view class_name,
                            std::string_view endpoint,
                            std::string_view topic) {
  if (!value.empty()) {
    return;
  }
  std::cerr << "[" << class_name << "]"
            << "[endpoint=" << endpoint << "]"
            << "[topic=" << topic << "]"
            << "[pid=" << CurrentPid() << "] "
            << "Invalid argument: " << field_name << " is empty."
            << std::endl;
  throw std::invalid_argument(std::string(field_name) + " is empty.");
}

}  // namespace

ZmqPublisherBytes::ZmqPublisherBytes(zmq::context_t& context,
                                     std::string endpoint,
                                     std::string topic)
    : context_(context),
      socket_(context_, zmq::socket_type::pub),
      endpoint_(std::move(endpoint)),
      topic_(std::move(topic)),
      lock_file_descriptor_(-1),
      lock_path_("") {
  ValidateNonEmptyOrThrow(endpoint_, "endpoint", kClassName, endpoint_, topic_);
  ValidateNonEmptyOrThrow(topic_, "topic", kClassName, endpoint_, topic_);

  // (1) lock 경로 결정
  lock_path_ = LockPathFromEndpoint();

  // (2) endpoint dir 생성 (ipc일 때만 의미)
  EnsureEndpointDirectoryExistsOrThrow();

  // (3) lock dir 생성 (ipc일 경우 sock dir와 동일, tcp일 경우 /tmp)
  EnsureLockDirectoryExistsOrThrow();

  // (4) flock 획득(중복 실행 차단) - 프로세스 수명 동안 유지
  AcquireProcessLockOrThrow();

  // (5) ipc인 경우에만 .sock cleanup (lock 획득 성공 후에만)
  CleanupIpcSocketFileOrThrow();

  // (6) 옵션 고정 설정
  ConfigureSocketOptionsOrThrow();

  // (7) bind
  try {
    socket_.bind(endpoint_);
  } catch (const zmq::error_t& e) {
    LogZmqErrorPrefix("bind failed", e);
    throw;
  }
}

ZmqPublisherBytes::~ZmqPublisherBytes() noexcept {
  try {
    socket_.close();
  } catch (const zmq::error_t& e) {
    LogZmqErrorPrefix("socket close failed (ignored)", e);
  } catch (...) {
    LogErrorPrefix("socket close failed (unknown, ignored)");
  }

  if (lock_file_descriptor_ >= 0) {
    if (::close(lock_file_descriptor_) != 0) {
      LogErrorPrefix("lock fd close failed (ignored)");
    }
    lock_file_descriptor_ = -1;
  }
}

void ZmqPublisherBytes::PublishBytes(const std::byte* payload_data,
                                     std::size_t payload_size) {
  if (payload_data == nullptr && payload_size != 0U) {
    LogErrorPrefix("PublishBytes invalid argument: payload_data is null");
    throw std::invalid_argument("payload_data is null");
  }

  try {
    zmq::message_t topic_frame(topic_.data(), topic_.size());

    zmq::message_t payload_frame(payload_size);
    if (payload_size != 0U) {
      std::memcpy(payload_frame.data(), payload_data, payload_size);
    }

    if (!socket_.send(topic_frame, zmq::send_flags::sndmore)) {
      LogErrorPrefix("send(topic) returned false");
      throw std::runtime_error("send(topic) returned false");
    }

    if (!socket_.send(payload_frame, zmq::send_flags::none)) {
      LogErrorPrefix("send(payload) returned false");
      throw std::runtime_error("send(payload) returned false");
    }
  } catch (const zmq::error_t& e) {
    LogZmqErrorPrefix("send failed", e);
    throw;
  }
}

void ZmqPublisherBytes::PublishBytes(std::vector<std::byte>&& payload) {
  try {
    auto* payload_owner = new std::vector<std::byte>(std::move(payload));

    zmq::message_t topic_frame(topic_.data(), topic_.size());

    zmq::message_t payload_frame(
        payload_owner->data(),
        payload_owner->size(),
        [](void* /*data*/, void* hint) {
          auto* owner = static_cast<std::vector<std::byte>*>(hint);
          delete owner;
        },
        payload_owner);

    if (!socket_.send(topic_frame, zmq::send_flags::sndmore)) {
      LogErrorPrefix("send(topic) returned false");
      throw std::runtime_error("send(topic) returned false");
    }

    if (!socket_.send(payload_frame, zmq::send_flags::none)) {
      LogErrorPrefix("send(payload) returned false");
      throw std::runtime_error("send(payload) returned false");
    }
  } catch (const zmq::error_t& e) {
    LogZmqErrorPrefix("send failed", e);
    throw;
  }
}

const std::string& ZmqPublisherBytes::Endpoint() const noexcept {
  return endpoint_;
}

const std::string& ZmqPublisherBytes::Topic() const noexcept {
  return topic_;
}

void ZmqPublisherBytes::AcquireProcessLockOrThrow() {
  lock_file_descriptor_ = ::open(lock_path_.c_str(), O_RDWR | O_CREAT, 0644);
  if (lock_file_descriptor_ < 0) {
    const int error_no = errno;
    std::cerr << "[" << kClassName << "]"
              << "[endpoint=" << endpoint_ << "]"
              << "[topic=" << topic_ << "]"
              << "[pid=" << CurrentPid() << "] "
              << "open(lock) failed: path=" << lock_path_
              << ", errno=" << error_no << " (" << ErrnoToString(error_no) << ")"
              << std::endl;
    throw std::runtime_error("open(lock) failed");
  }

  if (::flock(lock_file_descriptor_, LOCK_EX | LOCK_NB) != 0) {
    const int error_no = errno;
    std::cerr << "[" << kClassName << "]"
              << "[endpoint=" << endpoint_ << "]"
              << "[topic=" << topic_ << "]"
              << "[pid=" << CurrentPid() << "] "
              << "flock failed (already running?): path=" << lock_path_
              << ", errno=" << error_no << " (" << ErrnoToString(error_no) << ")"
              << std::endl;

    ::close(lock_file_descriptor_);
    lock_file_descriptor_ = -1;
    throw std::runtime_error("duplicate execution detected by flock");
  }
}

void ZmqPublisherBytes::CleanupIpcSocketFileOrThrow() {
  if (!IsIpcEndpoint()) {
    return;
  }

  const std::string ipc_path = IpcPathFromEndpointOrEmpty();
  if (ipc_path.empty()) {
    LogErrorPrefix("ipc endpoint parsing failed");
    throw std::invalid_argument("invalid ipc endpoint");
  }

  if (::unlink(ipc_path.c_str()) != 0) {
    const int error_no = errno;
    if (error_no == ENOENT) {
      return;
    }

    std::cerr << "[" << kClassName << "]"
              << "[endpoint=" << endpoint_ << "]"
              << "[topic=" << topic_ << "]"
              << "[pid=" << CurrentPid() << "] "
              << "unlink(.sock) failed: path=" << ipc_path
              << ", errno=" << error_no << " (" << ErrnoToString(error_no) << ")"
              << std::endl;
    throw std::runtime_error("unlink(.sock) failed");
  }
}

void ZmqPublisherBytes::ConfigureSocketOptionsOrThrow() {
  try {
    socket_.set(zmq::sockopt::linger, kLingerMillis);
    socket_.set(zmq::sockopt::sndhwm, kSendHighWaterMark);
    socket_.set(zmq::sockopt::sndtimeo, kSendTimeoutMillis);
  } catch (const zmq::error_t& e) {
    LogZmqErrorPrefix("set socket options failed", e);
    throw;
  }
}

void ZmqPublisherBytes::EnsureEndpointDirectoryExistsOrThrow() {
  if (!IsIpcEndpoint()) {
    return;
  }

  const std::string ipc_path = IpcPathFromEndpointOrEmpty();
  if (ipc_path.empty()) {
    LogErrorPrefix("ipc endpoint parsing failed (dir ensure)");
    throw std::invalid_argument("invalid ipc endpoint");
  }

  EnsureParentDirectoryExistsOrThrow(ipc_path);
}

void ZmqPublisherBytes::EnsureLockDirectoryExistsOrThrow() {
  EnsureParentDirectoryExistsOrThrow(lock_path_);
}

void ZmqPublisherBytes::EnsureParentDirectoryExistsOrThrow(const std::string& file_path) const {
  try {
    const std::filesystem::path path(file_path);
    const std::filesystem::path parent = path.parent_path();

    // parent가 비어있으면(예: "a.sock") 현재 작업 디렉터리로 간주 → 생성 불필요.
    if (parent.empty()) {
      return;
    }

    std::error_code error_code;
    const bool ok = std::filesystem::create_directories(parent, error_code);
    (void)ok;

    if (error_code) {
      std::cerr << "[" << kClassName << "]"
                << "[endpoint=" << endpoint_ << "]"
                << "[topic=" << topic_ << "]"
                << "[pid=" << CurrentPid() << "] "
                << "create_directories failed: dir=" << parent.string()
                << ", code=" << error_code.value()
                << " (" << error_code.message() << ")"
                << std::endl;
      throw std::runtime_error("create_directories failed");
    }
  } catch (const std::exception& e) {
    std::cerr << "[" << kClassName << "]"
              << "[endpoint=" << endpoint_ << "]"
              << "[topic=" << topic_ << "]"
              << "[pid=" << CurrentPid() << "] "
              << "create_directories exception: " << e.what()
              << std::endl;
    throw;
  }
}

bool ZmqPublisherBytes::IsIpcEndpoint() const noexcept {
  return endpoint_.rfind("ipc://", 0) == 0;
}

std::string ZmqPublisherBytes::IpcPathFromEndpointOrEmpty() const {
  if (!IsIpcEndpoint()) {
    return "";
  }
  return endpoint_.substr(std::string("ipc://").size());
}

std::string ZmqPublisherBytes::LockPathFromEndpoint() const {
  if (IsIpcEndpoint()) {
    const std::string ipc_path = IpcPathFromEndpointOrEmpty();
    if (ipc_path.empty()) {
      return "/tmp/zmq_publisher_invalid_ipc.lock";
    }
    return ipc_path + ".lock";
  }

  std::string sanitized = endpoint_;
  for (char& ch : sanitized) {
    const bool ok = (ch >= 'a' && ch <= 'z') ||
                    (ch >= 'A' && ch <= 'Z') ||
                    (ch >= '0' && ch <= '9') ||
                    ch == '_' || ch == '-' || ch == '.';
    if (!ok) {
      ch = '_';
    }
  }
  return std::string("/tmp/zmq_pub_lock_") + sanitized + ".lock";
}

void ZmqPublisherBytes::LogErrorPrefix(std::string_view message) const noexcept {
  std::cerr << "[" << kClassName << "]"
            << "[endpoint=" << endpoint_ << "]"
            << "[topic=" << topic_ << "]"
            << "[pid=" << CurrentPid() << "] "
            << message
            << std::endl;
}

void ZmqPublisherBytes::LogZmqErrorPrefix(std::string_view message,
                                         const zmq::error_t& error) const noexcept {
  std::cerr << "[" << kClassName << "]"
            << "[endpoint=" << endpoint_ << "]"
            << "[topic=" << topic_ << "]"
            << "[pid=" << CurrentPid() << "] "
            << message
            << ": zmq_errno=" << error.num()
            << " (" << error.what() << ")"
            << std::endl;
}

}  // namespace zmq_pub
