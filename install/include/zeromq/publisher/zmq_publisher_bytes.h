#ifndef ZMQ_PUBLISHER_BYTES_H_
#define ZMQ_PUBLISHER_BYTES_H_

#include <cstddef>
#include <string>
#include <string_view>
#include <vector>

#include <zmq.hpp>

namespace zmq_pub {

/**
 * @brief ZMQ PUB 소켓을 사용해 바이트 payload를 발행하는 퍼블리셔.
 * endpoint/topic을 보관하고, PublishBytes로 전달된 payload를 ZMQ 메시지로 송신한다.
 * 생성자에서 소켓 생성/옵션 설정/바인드 및 프로세스 단위 락 확보 등 초기화를 수행하고, 소멸자에서 리소스를 정리하는 RAII 패턴을 따른다.
 * IPC endpoint(ipc://...) 사용 시, endpoint에 대응하는 소켓 파일 및 락 파일 경로를 관리하며, 실행 환경에 따라 필요한 디렉터리 생성 및 파일 정리를 수행한다.
 * 프로세스 락을 통해 동일 endpoint에 대한 다중 프로세스 동시 바인드를 방지하며, 실패 시 예외를 통해 호출자에게 즉시 알린다.
 * @note 전송 실패/바인드 실패 등 ZMQ 관련 오류는 예외로 보고되며, 상위 계층에서 재시도/재생성 정책을 결정하는 것을 전제로 한다.
 */
class ZmqPublisherBytes {
 public:
  /**
   * @brief ZmqPublisherBytes 생성자.
   * context를 사용해 PUB 소켓을 생성하고 endpoint/topic을 설정한다.
   * IPC endpoint인 경우 endpoint/lock 경로의 parent 디렉터리 존재를 보장하고, 필요 시 생성한다.
   * 프로세스 락을 획득하여 동일 endpoint에 대한 중복 실행을 방지한다.
   * IPC endpoint인 경우 이전 실행의 잔존 소켓 파일 정리 등 cleanup을 수행할 수 있다.
   * 소켓 옵션을 설정한 뒤 endpoint에 bind/연결한다(구체 정책은 구현에 따름).
   * @param context ZMQ 컨텍스트 참조(외부 소유).
   * @param endpoint 바인드/연결에 사용할 endpoint 문자열(예: tcp://..., ipc://...).
   * @param topic 발행 토픽 문자열(구독 측 필터링에 사용).
   * @throw std::exception 디렉터리 생성, 락 획득, cleanup, 소켓 옵션 설정, bind/연결 과정에서 실패 시.
   * @note endpoint/topic은 생성 이후 고정되며, 이 인스턴스는 단일 endpoint/topic 발행에 사용된다.
   */
  ZmqPublisherBytes(zmq::context_t& context,
                    std::string endpoint,
                    std::string topic);

  /**
   * @brief ZmqPublisherBytes 소멸자.
   * 소켓 및 락 파일 디스크립터 등 내부 리소스를 정리한다.
   * 예외가 외부로 전파되지 않도록 noexcept로 선언된다.
   * @note IPC endpoint인 경우, 구현 정책에 따라 잔존 소켓 파일 정리를 수행할 수 있다.
   */
  ~ZmqPublisherBytes() noexcept;

  ZmqPublisherBytes(const ZmqPublisherBytes&) = delete;
  ZmqPublisherBytes& operator=(const ZmqPublisherBytes&) = delete;

  ZmqPublisherBytes(ZmqPublisherBytes&&) = delete;
  ZmqPublisherBytes& operator=(ZmqPublisherBytes&&) = delete;

  /**
   * @brief 바이트 버퍼를 발행한다.
   * payload_data/payload_size로 전달된 내용을 ZMQ 메시지로 송신한다.
   * @param payload_data 전송할 payload 시작 주소.
   * @param payload_size 전송할 payload 크기(바이트).
   * @throw zmq::error_t 또는 std::exception 전송 과정에서 ZMQ 오류가 발생한 경우.
   * @note 호출자는 payload_data가 유효한 메모리를 가리키도록 보장해야 한다.
   */
  void PublishBytes(const std::byte* payload_data, std::size_t payload_size);

  /**
   * @brief 바이트 벡터를 발행한다(소유권 이동).
   * 전달된 벡터를 이동 받아 전송에 사용하며, 호출자 측 복사를 최소화한다.
   * @param payload 전송할 payload(이동 후 원본은 사용 불가/비어있을 수 있음).
   * @throw zmq::error_t 또는 std::exception 전송 과정에서 ZMQ 오류가 발생한 경우.
   */
  void PublishBytes(std::vector<std::byte>&& payload);

  /**
   * @brief 설정된 endpoint를 반환한다.
   * @return endpoint 문자열 참조.
   */
  const std::string& Endpoint() const noexcept;

  /**
   * @brief 설정된 topic을 반환한다.
   * @return topic 문자열 참조.
   */
  const std::string& Topic() const noexcept;

 private:
  static constexpr std::string_view kClassName = "ZmqPublisherBytes"; ///< 로깅에 사용되는 클래스 이름.

  /**
   * @brief 프로세스 단위 락을 획득한다.
   * 동일 endpoint에 대해 동시에 실행되는 다른 프로세스가 있을 경우 실패할 수 있다.
   * @throw std::exception 락 획득 실패 시.
   */
  void AcquireProcessLockOrThrow();

  /**
   * @brief IPC endpoint 사용 시 잔존 소켓 파일을 정리한다.
   * 이전 실행이 비정상 종료된 경우 남아있는 ipc 소켓 파일이 bind를 방해할 수 있으므로 이를 제거한다.
   * @throw std::exception 정리 작업 실패 시.
   */
  void CleanupIpcSocketFileOrThrow();

  /**
   * @brief ZMQ 소켓 옵션을 설정한다.
   * LINGER, SNDHWM 등 필요한 옵션을 설정할 수 있다(구체 항목은 구현에 따름).
   * @throw zmq::error_t 또는 std::exception 옵션 설정 실패 시.
   */
  void ConfigureSocketOptionsOrThrow();

  /**
   * @brief endpoint/lock 경로의 parent 디렉터리를 보장한다.
   * IPC endpoint의 경우 파일 시스템 경로를 사용하므로, 필요한 디렉터리 생성/검증을 수행한다.
   * @throw std::exception 디렉터리 생성/검증 실패 시.
   */
  void EnsureEndpointDirectoryExistsOrThrow();

  /**
   * @brief 락 파일 경로의 parent 디렉터리를 보장한다.
   * @throw std::exception 디렉터리 생성/검증 실패 시.
   */
  void EnsureLockDirectoryExistsOrThrow();

  /**
   * @brief endpoint/lock에 필요한 모든 parent 디렉터리를 보장한다.
   * @throw std::exception 디렉터리 생성/검증 실패 시.
   */
  void EnsureParentDirectoryExistsOrThrow(const std::string& file_path) const;

  /**
   * @brief IPC endpoint인지 여부를 반환한다.
   * @return endpoint가 ipc:// 스킴이면 true.
   */
  bool IsIpcEndpoint() const noexcept;

  /**
   * @brief IPC endpoint에서 파일 시스템 경로를 추출한다.
   * ipc://... 형식인 경우 해당 경로를 반환하고, 그렇지 않으면 빈 문자열을 반환한다.
   * @return IPC 소켓 파일 경로 또는 빈 문자열.
   */
  std::string IpcPathFromEndpointOrEmpty() const;

  /**
   * @brief endpoint에 대응하는 락 파일 경로를 생성한다.
   * IPC endpoint 기반으로 lock_path_를 계산한다(구체 형식은 구현에 따름).
   * @return 락 파일 경로 문자열.
   */
  std::string LockPathFromEndpoint() const;

  /**
   * @brief 공통 오류 로그 prefix 출력.
   * endpoint/topic 정보를 포함한 prefix를 stderr로 출력한다.
   * @param message 출력할 메시지 본문.
   */
  void LogErrorPrefix(std::string_view message) const noexcept;

  /**
   * @brief ZMQ 오류 정보를 포함한 로그 prefix 출력.
   * endpoint/topic 및 zmq::error_t 정보를 포함해 stderr로 출력한다.
   * @param message 출력할 메시지 본문.
   * @param error ZMQ 오류 객체.
   */
  void LogZmqErrorPrefix(std::string_view message, const zmq::error_t& error) const noexcept;

  zmq::context_t& context_;   ///< 외부에서 소유되는 ZMQ 컨텍스트 참조.
  zmq::socket_t socket_;      ///< PUB 소켓.
  std::string endpoint_;      ///< 바인드/연결 endpoint.
  std::string topic_;         ///< 발행 토픽.

  int lock_file_descriptor_;  ///< 프로세스 락 파일 디스크립터.
  std::string lock_path_;     ///< 프로세스 락 파일 경로.
};

}  // namespace zmq_pub

#endif  // ZMQ_PUBLISHER_BYTES_H_
