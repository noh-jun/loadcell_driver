#ifndef ZMQ_PUBLISHER_MSGPACK_HPP_
#define ZMQ_PUBLISHER_MSGPACK_HPP_

#include <cstddef>
#include <cstring>
#include <iostream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <unistd.h>      // getpid
#include <msgpack.hpp>   // msgpack::pack, sbuffer

#include "zmq_publisher_bytes.h"

namespace zmq_pub {

/**
 * @brief MsgPack 기반 Publisher (단일 타입 전용).
 * 템플릿 파라미터 T payload를 MsgPack으로 직렬화한 뒤 바이트 배열로 전송한다.
 * 1 인스턴스는 1 endpoint(bind) 및 1 topic에 고정되며, 직렬화 모델(T)도 인스턴스 단위로 고정된다.
 * 직렬화 실패는 이 클래스에서 로그(stderr) 출력 후 예외를 재전파한다.
 * 전송/소켓 관리(zmq send/bind/lock/cleanup) 실패 처리는 하위 ZmqPublisherBytes가 담당한다.
 * @tparam T MsgPack 직렬화가 가능한 payload 타입.
 */
template <typename T>
class MsgPackPublisher {
 public:
  /**
   * @brief MsgPackPublisher 생성자.
   * 하위 ZmqPublisherBytes를 생성하여 endpoint/topic에 바인드/설정한다(구체 동작은 ZmqPublisherBytes 구현에 따름).
   * @param context ZMQ 컨텍스트 참조(외부 소유).
   * @param endpoint 바인드/연결에 사용할 endpoint 문자열.
   * @param topic 발행 토픽 문자열.
   * @throw std::exception 하위 퍼블리셔 생성/바인드 과정에서 예외가 발생할 수 있다(구체 예외는 하위 구현에 따름).
   */
  MsgPackPublisher(zmq::context_t& context, std::string endpoint, std::string topic)
      : publisher_(context, std::move(endpoint), std::move(topic)) {}

  /**
   * @brief MsgPackPublisher 소멸자.
   * 하위 퍼블리셔 리소스를 정리한다(RAII).
   */
  ~MsgPackPublisher() = default;

  MsgPackPublisher(const MsgPackPublisher&) = delete;
  MsgPackPublisher& operator=(const MsgPackPublisher&) = delete;
  MsgPackPublisher(MsgPackPublisher&&) = delete;
  MsgPackPublisher& operator=(MsgPackPublisher&&) = delete;

  /**
   * @brief T payload를 MsgPack으로 직렬화한 뒤 publish한다.
   * payload를 msgpack::pack으로 sbuffer에 직렬화한다.
   * 직렬화 결과를 std::vector<std::byte>로 복사한 뒤, 하위 publisher_.PublishBytes로 전송한다.
   * @param payload 전송할 payload.
   * @throw std::exception MsgPack 직렬화 실패 시 예외를 던진다(로그 출력 후 재전파).
   * @throw std::exception 하위 전송 경로(ZmqPublisherBytes)에서 예외가 발생하면 그대로 전파된다.
   * @note 직렬화 실패 시 "[MsgPackPublisher][endpoint=...][topic=...][pid=...]" 형태의 prefix로 stderr에 로그를 남긴다.
   */
  void Publish(const T& payload) {
    std::vector<std::byte> packed = SerializeOrThrow(payload);
    publisher_.PublishBytes(std::move(packed));
  }

  /**
   * @brief 현재 설정된 endpoint를 반환한다.
   * @return endpoint 문자열 참조.
   */
  const std::string& Endpoint() const noexcept { return publisher_.Endpoint(); }

  /**
   * @brief 현재 설정된 topic을 반환한다.
   * @return topic 문자열 참조.
   */
  const std::string& Topic() const noexcept { return publisher_.Topic(); }

 private:
  static constexpr std::string_view kClassName = "MsgPackPublisher"; ///< 로깅에 사용되는 클래스 이름.

  /**
   * @brief 현재 프로세스 PID 반환.
   * @return pid_t 현재 프로세스 ID.
   */
  static pid_t CurrentPid() noexcept {
    return ::getpid();
  }

  /**
   * @brief 예외 객체가 없는 오류 로그 prefix 출력.
   * endpoint/topic/pid 정보를 포함한 공통 prefix를 구성하여 stderr로 출력한다.
   * @param message 출력할 메시지 본문.
   */
  void LogErrorPrefix(std::string_view message) const noexcept {
    std::cerr << "[" << kClassName << "]"
              << "[endpoint=" << publisher_.Endpoint() << "]"
              << "[topic=" << publisher_.Topic() << "]"
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
              << "[endpoint=" << publisher_.Endpoint() << "]"
              << "[topic=" << publisher_.Topic() << "]"
              << "[pid=" << CurrentPid() << "] "
              << message << ": " << exception.what()
              << std::endl;
  }

  /**
   * @brief payload를 MsgPack으로 직렬화하여 바이트 배열로 반환한다.
   * msgpack::sbuffer에 직렬화 후, 동일 크기의 std::vector<std::byte>를 할당하여 내용을 복사한다.
   * buffer 크기가 0인 경우 memcpy를 수행하지 않으며, 빈 벡터를 반환한다.
   * @param payload 직렬화할 payload.
   * @return 직렬화된 바이트 배열.
   * @throw std::exception 직렬화 과정에서 std::exception 파생 예외가 발생하면 로그 출력 후 재전파한다.
   * @throw ... 알 수 없는 예외가 발생하면 로그 출력 후 재전파한다.
   */
  std::vector<std::byte> SerializeOrThrow(const T& payload) {
    try {
      msgpack::sbuffer buffer;
      msgpack::pack(buffer, payload);

      std::vector<std::byte> bytes;
      bytes.resize(static_cast<std::size_t>(buffer.size()));
      if (buffer.size() != 0U) {
        std::memcpy(bytes.data(), buffer.data(), buffer.size());
      }
      return bytes;
    } catch (const std::exception& e) {
      LogExceptionPrefix("msgpack serialize failed", e);
      throw;
    } catch (...) {
      LogErrorPrefix("msgpack serialize failed: unknown exception");
      throw;
    }
  }

  ZmqPublisherBytes publisher_; ///< 바이트 전송 및 ZMQ 소켓/동기화/정리 담당.
};

}  // namespace zmq_pub

#endif  // ZMQ_PUBLISHER_MSGPACK_HPP_
