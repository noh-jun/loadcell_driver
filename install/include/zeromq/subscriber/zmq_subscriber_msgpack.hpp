// zmq_subscriber_msgpack.hpp
#ifndef ZMQ_SUBSCRIBER_MSGPACK_HPP_
#define ZMQ_SUBSCRIBER_MSGPACK_HPP_

#include <cstddef>
#include <exception>
#include <iostream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <unistd.h>     // getpid
#include <msgpack.hpp>  // unpack, object_handle

#include "zmq_subscriber_bytes.h"

namespace zmq_pub {

/**
 * @brief MsgPack 기반 Subscriber (단일 타입 전용).
 *
 * ZmqSubscriberBytes가 수신한 bytes payload를 디스패치 스레드에서 pop하여 MsgPack으로 역직렬화한 뒤 사용자 콜백(on_message_)에 전달한다.
 *
 * 1 인스턴스는 1 endpoint(connect), 1 topic, 1 역직렬화 모델(T)에 고정된다.
 *
 * 역직렬화 또는 사용자 콜백에서 발생하는 예외는 스레드 밖으로 throw되지 않으며, stderr 로그 출력 후 error callback 경로로 전달된다.
 *
 * Error callback 정책:
 *  - true  반환 → continue
 *  - false 반환 → stop
 *
 * bytes layer 오류 또한 동일하게 error callback으로 전달된다.
 *
 * @tparam T MsgPack 역직렬화가 가능한 메시지 타입.
 */
template <typename T>
class MsgPackSubscriber {
 public:
  /**
   * @brief 메시지 수신 콜백 타입.
   * 역직렬화된 T 메시지를 rvalue로 전달받는다.
   * @note 콜백은 내부 디스패치 스레드에서 호출된다.
   */
  using MessageCallback = std::function<void(T&& message)>;

  /**
   * @brief 오류 콜백 타입.
   * 오류 메시지를 전달받고, true를 반환하면 계속 진행하며 false를 반환하면 subscriber를 중지한다.
   * @note 콜백은 내부 스레드에서 호출될 수 있으며, 예외를 던지지 않는 것을 권장한다.
   */
  using ErrorCallback = std::function<bool(const std::string& error_message)>;

  /**
   * @brief MsgPackSubscriber 생성자.
   * bytes subscriber를 생성하여 endpoint/topic으로 connect 및 내부 큐를 구성한다(구체 동작은 ZmqSubscriberBytes 구현에 따름).
   * bytes 메시지가 도착하면 OnBytesMessage로 전달되어 역직렬화 후 on_message_가 호출된다.
   * bytes layer 오류는 OnBytesError로 전달되며, on_error_ 반환값에 따라 continue/stop 정책이 적용된다.
   * @param context ZMQ 컨텍스트 참조(외부 소유).
   * @param endpoint connect할 endpoint 문자열.
   * @param topic subscribe할 토픽 문자열.
   * @param max_queue_size bytes subscriber 내부 큐의 최대 크기.
   * @param on_message 역직렬화된 메시지 수신 콜백(필수).
   * @param on_error 오류 콜백(선택). 미등록 시 오류 발생 시 stop된다.
   * @throw std::invalid_argument on_message가 null인 경우.
   * @throw std::exception 하위 subscriber 생성/연결 과정에서 예외가 발생할 수 있다(구체 예외는 하위 구현에 따름).
   * @note on_message는 반드시 유효해야 하며, 본 생성자에서 즉시 검증한다.
   */
  MsgPackSubscriber(zmq::context_t& context,
                    std::string endpoint,
                    std::string topic,
                    std::size_t max_queue_size,
                    MessageCallback on_message,
                    ErrorCallback on_error = nullptr)
      : endpoint_(endpoint),
        topic_(topic),
        on_message_(std::move(on_message)),
        on_error_(std::move(on_error)),
        bytes_subscriber_(context,
                          std::move(endpoint),
                          std::move(topic),
                          max_queue_size,
                          [this](std::vector<std::byte>&& payload) {
                            this->OnBytesMessage(std::move(payload));
                          },
                          [this](const std::string& error_message) {
                            return this->OnBytesError(error_message);
                          }) {
    if (!on_message_) {
      LogErrorPrefix("Invalid argument: on_message callback is null");
      throw std::invalid_argument("on_message callback is null");
    }
  }

  /**
   * @brief MsgPackSubscriber 소멸자.
   * 하위 bytes subscriber 리소스를 정리한다(RAII).
   * @note 소멸자에서 예외가 전파되지 않도록 noexcept를 유지한다.
   */
  ~MsgPackSubscriber() noexcept = default;

  MsgPackSubscriber(const MsgPackSubscriber&) = delete;
  MsgPackSubscriber& operator=(const MsgPackSubscriber&) = delete;
  MsgPackSubscriber(MsgPackSubscriber&&) = delete;
  MsgPackSubscriber& operator=(MsgPackSubscriber&&) = delete;

  /**
   * @brief subscriber를 중지한다.
   * 하위 bytes subscriber에 stop 요청을 전달한다.
   * @note stop 이후에는 메시지 콜백이 더 이상 호출되지 않도록 설계된다(구체 동작은 하위 구현에 따름).
   */
  void Stop() noexcept { bytes_subscriber_.Stop(); }

  /**
   * @brief 현재 설정된 endpoint를 반환한다.
   * @return endpoint 문자열 참조.
   */
  const std::string& Endpoint() const noexcept { return bytes_subscriber_.Endpoint(); }

  /**
   * @brief 현재 설정된 topic을 반환한다.
   * @return topic 문자열 참조.
   */
  const std::string& Topic() const noexcept { return bytes_subscriber_.Topic(); }

 private:
  static constexpr std::string_view kClassName = "MsgPackSubscriber"; ///< 로깅에 사용되는 클래스 이름.

  /**
   * @brief 현재 프로세스 PID 반환.
   * @return pid_t 현재 프로세스 ID.
   */
  static pid_t CurrentPid() noexcept { return ::getpid(); }

  /**
   * @brief 예외 객체가 없는 오류 로그 prefix 출력.
   * endpoint/topic/pid 정보를 포함한 공통 prefix를 구성하여 stderr로 출력한다.
   * @param message 출력할 메시지 본문.
   */
  void LogErrorPrefix(std::string_view message) const noexcept {
    std::cerr << "[" << kClassName << "]"
              << "[endpoint=" << bytes_subscriber_.Endpoint() << "]"
              << "[topic=" << bytes_subscriber_.Topic() << "]"
              << "[pid=" << CurrentPid() << "] "
              << message
              << std::endl;
  }

  /**
   * @brief 예외 객체를 포함한 오류 로그 prefix 출력.
   * endpoint/topic/pid 정보를 포함한 공통 prefix 및 exception.what()을 stderr로 출력한다.
   * @param message 출력할 메시지 본문.
   * @param exception 발생한 예외 객체.
   */
  void LogExceptionPrefix(std::string_view message,
                          const std::exception& exception) const noexcept {
    std::cerr << "[" << kClassName << "]"
              << "[endpoint=" << bytes_subscriber_.Endpoint() << "]"
              << "[topic=" << bytes_subscriber_.Topic() << "]"
              << "[pid=" << CurrentPid() << "] "
              << message << ": " << exception.what()
              << std::endl;
  }

  /**
   * @brief bytes layer 오류를 상위 error callback으로 전달한다.
   * on_error_가 등록되어 있지 않으면 stop을 유도하기 위해 false를 반환한다.
   * on_error_ 호출 중 예외가 발생하면 로그를 남기고 stop을 유도하기 위해 false를 반환한다.
   * @param error_message bytes layer에서 전달된 오류 메시지.
   * @return true면 계속 진행, false면 stop.
   */
  bool OnBytesError(const std::string& error_message) noexcept {
    // bytes layer error: 그대로 상위 error callback으로 전달
    if (!on_error_) {
      // 등록 없으면 stop
      return false;
    }

    try {
      return on_error_(error_message);
    } catch (const std::exception& e) {
      LogExceptionPrefix("on_error callback exception", e);
      return false;
    } catch (...) {
      LogErrorPrefix("on_error callback unknown exception");
      return false;
    }
  }

  /**
   * @brief bytes payload 수신 처리(역직렬화 및 사용자 콜백 디스패치).
   * payload를 MsgPack으로 역직렬화하여 T로 변환한 뒤 on_message_를 호출한다.
   * 스레드 내부에서 호출되므로 throw하지 않으며, 모든 예외는 stderr 로깅 후 error callback 경로로 전달한다.
   * on_error_가 미등록된 경우에는 stop을 유도하기 위해 bytes_subscriber_.Stop()을 호출한다.
   * @param payload 수신된 payload 바이트 벡터(이동 전달).
   */
  void OnBytesMessage(std::vector<std::byte>&& payload) noexcept {
    // 스레드 내부이므로 throw 금지. 모든 예외는 stderr + error callback으로 제어.
    try {
      T message = DeserializeOrThrow(payload);
      on_message_(std::move(message));
    } catch (const std::exception& e) {
      LogExceptionPrefix("msgpack deserialize or on_message failed", e);
      const std::string error_message = std::string("msgpack deserialize/on_message failed: ") + e.what();

      if (!on_error_) {
        // 등록 없으면 stop을 유도: bytes subscriber의 error callback이 false를 반환하도록.
        // 여기서는 직접 stop 요청을 하지 않고, error callback 경로로 전달.
        // (bytes subscriber는 callback return 값에 따라 stop/continue 결정)
        bytes_subscriber_.Stop();
        return;
      }

      const bool should_continue = OnBytesError(error_message);
      if (!should_continue) {
        bytes_subscriber_.Stop();
      }
    } catch (...) {
      LogErrorPrefix("msgpack deserialize or on_message failed: unknown exception");
      const std::string error_message = "msgpack deserialize/on_message failed: unknown exception";

      if (!on_error_) {
        bytes_subscriber_.Stop();
        return;
      }

      const bool should_continue = OnBytesError(error_message);
      if (!should_continue) {
        bytes_subscriber_.Stop();
      }
    }
  }

  /**
   * @brief payload를 MsgPack으로 역직렬화하여 T로 변환한다.
   * msgpack::unpack을 호출하여 object_handle을 생성하고, object.as<T>()로 타입 변환을 수행한다.
   * payload가 비어 있으면 size 0으로 unpack을 수행한다.
   * @param payload 수신된 바이트 payload.
   * @return 역직렬화된 T 메시지.
   * @throw std::exception object.as<T>()에 필요한 adaptor가 없거나 역직렬화에 실패하면 예외가 발생한다.
   */
  T DeserializeOrThrow(const std::vector<std::byte>& payload) {
    // msgpack unpack expects char*
    const char* data = nullptr;
    std::size_t size = payload.size();

    if (!payload.empty()) {
      data = reinterpret_cast<const char*>(payload.data());
    } else {
      data = "";
      size = 0U;
    }

    msgpack::object_handle handle = msgpack::unpack(data, size);
    msgpack::object object = handle.get();

    // object.as<T>()는 adaptor가 없으면 예외 발생
    return object.as<T>();
  }

  // endpoint/topic은 생성자에서 move로 bytes_subscriber_로 들어가므로,
  // 멤버로 별도 유지할 필요는 없지만, 디버그 의도를 명확히 하기 위해 남김(사용은 안 함).
  std::string endpoint_;
  std::string topic_;

  MessageCallback on_message_; ///< 메시지 수신 콜백(필수).
  ErrorCallback on_error_;     ///< 오류 콜백(선택, 미등록 시 오류 발생 시 stop).

  ZmqSubscriberBytes bytes_subscriber_; ///< bytes 수신/큐/디스패치 및 stop 제어 담당.
};

}  // namespace zmq_pub

#endif  // ZMQ_SUBSCRIBER_MSGPACK_HPP_
