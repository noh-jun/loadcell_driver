#include "zmq_subscriber_bytes.h"

#include <cerrno>
#include <chrono>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <utility>

#include <unistd.h>

namespace zmq_pub {
namespace {

constexpr int kReceiveTimeoutMillis = 100;  // stop 요청 반영을 위해 폴링
constexpr int kReceiveHighWaterMark = 0;    // 0이면 libzmq 기본값 사용(정책 미확정이라 고정하지 않음)

std::string ErrnoToString(int error_no) {
  return std::string(::strerror(error_no));
}

}  // namespace

int ZmqSubscriberBytes::CurrentPid() noexcept {
  return static_cast<int>(::getpid());
}

ZmqSubscriberBytes::ZmqSubscriberBytes(zmq::context_t& context,
                                       std::string endpoint,
                                       std::string topic,
                                       std::size_t max_queue_size,
                                       MessageCallback on_message,
                                       ErrorCallback on_error)
    : context_(context),
      socket_(context_, zmq::socket_type::sub),
      endpoint_(std::move(endpoint)),
      topic_(std::move(topic)),
      max_queue_size_(max_queue_size),
      on_message_(std::move(on_message)),
      on_error_(std::move(on_error)),
      stop_requested_(false),
      queue_mutex_(),
      queue_condition_(),
      queue_(),
      recv_thread_(),
      dispatch_thread_() {
  if (endpoint_.empty()) {
    LogErrorPrefix("Invalid argument: endpoint is empty");
    throw std::invalid_argument("endpoint is empty");
  }
  if (topic_.empty()) {
    LogErrorPrefix("Invalid argument: topic is empty");
    throw std::invalid_argument("topic is empty");
  }
  if (max_queue_size_ == 0U) {
    LogErrorPrefix("Invalid argument: max_queue_size must be > 0");
    throw std::invalid_argument("max_queue_size must be > 0");
  }
  if (!on_message_) {
    LogErrorPrefix("Invalid argument: on_message callback is null");
    throw std::invalid_argument("on_message callback is null");
  }

  try {
    socket_.set(zmq::sockopt::rcvtimeo, kReceiveTimeoutMillis);

    // rcvhwm 정책은 아직 사용자 고정 지시가 없으므로 기본값 유지.
    // 필요 시 아래 주석 해제:
    // socket_.set(zmq::sockopt::rcvhwm, kReceiveHighWaterMark);

    // topic 구독
    socket_.set(zmq::sockopt::subscribe, topic_);

    // connect
    socket_.connect(endpoint_);
  } catch (const zmq::error_t& e) {
    LogZmqErrorPrefix("subscriber socket setup/connect failed", e);
    throw;
  }

  StartThreadsOrThrow();
}

ZmqSubscriberBytes::~ZmqSubscriberBytes() noexcept {
  Stop();

  try {
    socket_.close();
  } catch (const zmq::error_t& e) {
    LogZmqErrorPrefix("socket close failed (ignored)", e);
  } catch (...) {
    LogErrorPrefix("socket close failed (unknown, ignored)");
  }
}

void ZmqSubscriberBytes::Stop() noexcept {
  // (A) 요청은 항상 true로 고정 (idempotent)
  stop_requested_.store(true);

  // (B) wait 깨우기
  queue_condition_.notify_all();

  // (C) 자기 자신 join 금지
  const std::thread::id self_id = std::this_thread::get_id();

  if (recv_thread_.joinable() && recv_thread_.get_id() != self_id) {
    recv_thread_.join();
  }
  if (dispatch_thread_.joinable() && dispatch_thread_.get_id() != self_id) {
    dispatch_thread_.join();
  }
}

const std::string& ZmqSubscriberBytes::Endpoint() const noexcept {
  return endpoint_;
}

const std::string& ZmqSubscriberBytes::Topic() const noexcept {
  return topic_;
}

void ZmqSubscriberBytes::StartThreadsOrThrow() {
  try {
    recv_thread_ = std::thread([this]() { RecvLoop(); });
    dispatch_thread_ = std::thread([this]() { DispatchLoop(); });
  } catch (const std::exception& e) {
    LogErrorPrefix(std::string("thread start failed: ") + e.what());
    stop_requested_.store(true);
    queue_condition_.notify_all();
    if (recv_thread_.joinable()) {
      recv_thread_.join();
    }
    if (dispatch_thread_.joinable()) {
      dispatch_thread_.join();
    }
    throw;
  }
}

void ZmqSubscriberBytes::RecvLoop() noexcept {
  while (!stop_requested_.load()) {
    try {
      zmq::message_t topic_frame;
      zmq::message_t payload_frame;

      const auto topic_result = socket_.recv(topic_frame, zmq::recv_flags::none);
      if (!topic_result.has_value()) {
        // timeout: stop 체크하고 계속
        continue;
      }

      const auto payload_result = socket_.recv(payload_frame, zmq::recv_flags::none);
      if (!payload_result.has_value()) {
        // 비정상 멀티파트. 다음 루프로.
        if (!ShouldContinueOnError("recv(payload) timeout or missing frame")) {
          stop_requested_.store(true);
          queue_condition_.notify_all();
          return;
        }
        continue;
      }

      // topic 필터는 libzmq subscribe가 수행하지만, 방어적으로 한번 더 확인 가능.
      // (subscribe가 prefix 매칭이므로 topic 정책이 완전 일치인지 확인하는 용도)
      const std::string_view received_topic(
          static_cast<const char*>(topic_frame.data()),
          topic_frame.size());

      if (received_topic != topic_) {
        // 정책상 "1 object = 1 topic"이므로 다른 토픽은 무시.
        continue;
      }

      std::vector<std::byte> payload;
      payload.resize(payload_frame.size());
      if (payload_frame.size() != 0U) {
        std::memcpy(payload.data(), payload_frame.data(), payload_frame.size());
      }

      EnqueueOrDropOldest(std::move(payload));
    } catch (const zmq::error_t& e) {
      LogZmqErrorPrefix("recv failed", e);
      if (!ShouldContinueOnError(std::string("zmq recv failed: ") + e.what())) {
        stop_requested_.store(true);
        queue_condition_.notify_all();
        return;
      }
    } catch (const std::exception& e) {
      LogErrorPrefix(std::string("recv loop exception: ") + e.what());
      if (!ShouldContinueOnError(std::string("recv loop exception: ") + e.what())) {
        stop_requested_.store(true);
        queue_condition_.notify_all();
        return;
      }
    } catch (...) {
      LogErrorPrefix("recv loop unknown exception");
      if (!ShouldContinueOnError("recv loop unknown exception")) {
        stop_requested_.store(true);
        queue_condition_.notify_all();
        return;
      }
    }
  }
}

void ZmqSubscriberBytes::DispatchLoop() noexcept {
  std::vector<std::byte> payload;

  while (true) {
    // mutex lock 구간
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      queue_condition_.wait(
          lock, [this]() { return stop_requested_.load() || !queue_.empty(); });

      if (queue_.empty()) {
        if (stop_requested_.load())
          return;

        continue;
      }

      payload.clear();
      payload = std::move(queue_.front());
      queue_.pop_front();
    }

    try {
      on_message_(std::move(payload));
    } catch (const std::exception &e) {
      LogErrorPrefix(std::string("on_message callback exception: ") + e.what());
      if (!ShouldContinueOnError(std::string("on_message exception: ") +
                                 e.what())) {
        stop_requested_.store(true);
        queue_condition_.notify_all();
        return;
      }
    } catch (...) {
      LogErrorPrefix("on_message callback unknown exception");
      if (!ShouldContinueOnError("on_message unknown exception")) {
        stop_requested_.store(true);
        queue_condition_.notify_all();
        return;
      }
    }
  }
}

void ZmqSubscriberBytes::EnqueueOrDropOldest(std::vector<std::byte>&& payload) {
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);

    if (queue_.size() >= max_queue_size_) {
      // oldest drop
      queue_.pop_front();
    }
    queue_.push_back(std::move(payload));
  }
  queue_condition_.notify_one();
}

bool ZmqSubscriberBytes::ShouldContinueOnError(const std::string& error_message) noexcept {
  if (!on_error_) {
    // error callback이 없으면 기본 정책: stop
    return false;
  }

  try {
    return on_error_(error_message);
  } catch (const std::exception& e) {
    LogErrorPrefix(std::string("on_error callback exception: ") + e.what());
    return false;
  } catch (...) {
    LogErrorPrefix("on_error callback unknown exception");
    return false;
  }
}

void ZmqSubscriberBytes::LogErrorPrefix(std::string_view message) const noexcept {
  std::cerr << "[" << kClassName << "]"
            << "[endpoint=" << endpoint_ << "]"
            << "[topic=" << topic_ << "]"
            << "[pid=" << CurrentPid() << "] "
            << message
            << std::endl;
}

void ZmqSubscriberBytes::LogZmqErrorPrefix(std::string_view message,
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
