#ifndef ZMQ_SUBSCRIBER_BYTES_H_
#define ZMQ_SUBSCRIBER_BYTES_H_

#include <atomic>
#include <cstddef>
#include <condition_variable>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include <zmq.hpp>

namespace zmq_pub {

/**
 * @brief ZMQ SUB 소켓으로 수신한 payload 바이트를 큐잉하고 콜백으로 전달한다.
 *
 * 수신 스레드에서 SUB 소켓으로 메시지를 수신해 payload를 내부 큐에 적재한다.
 * 디스패치 스레드에서 큐에 쌓인 payload를 꺼내 on_message_ 콜백으로 전달한다.
 * 큐 크기는 max_queue_size_로 제한되며, 초과 시 가장 오래된 항목을 폐기하여 최신 데이터 전달을 우선한다.
 * 오류 발생 시 on_error_ 콜백으로 오류 메시지를 전달하고, 콜백 반환값에 따라 루프를 지속하거나 중단한다.
 * Stop() 또는 소멸자에서 stop_requested_를 설정하고 스레드를 join하여 안전하게 종료한다.
 *
 * @note context_는 외부에서 소유되며 본 객체는 참조만 유지한다.
 * @note Stop()은 종료를 요청하는 역할만 수행하며, 실제 종료 완료는 join 이후 보장된다.
 */
class ZmqSubscriberBytes {
 public:
  using MessageCallback = std::function<void(std::vector<std::byte>&& payload)>;

  using ErrorCallback = std::function<bool(const std::string& error_message)>;

  /**
   * @brief endpoint/topic 및 콜백을 설정하고 내부 스레드를 기동한다.
   *
   * @param context ZMQ 컨텍스트 참조(외부 소유).
   * @param endpoint SUB 소켓이 연결할 endpoint 문자열.
   * @param topic 구독할 토픽 문자열.
   * @param max_queue_size 내부 큐 최대 크기.
   * @param on_message payload 수신 시 호출되는 콜백.
   * @param on_error 오류 발생 시 호출되는 콜백이며, true를 반환하면 계속 진행하고 false를 반환하면 중단한다.
   * @throw std::exception 소켓 설정 또는 스레드 기동 실패 시.
   */
  ZmqSubscriberBytes(zmq::context_t& context,
                     std::string endpoint,
                     std::string topic,
                     std::size_t max_queue_size,
                     MessageCallback on_message,
                     ErrorCallback on_error = nullptr);

  /**
   * @brief stop 요청 후 스레드를 join하여 종료한다.
   */
  ~ZmqSubscriberBytes() noexcept;

  ZmqSubscriberBytes(const ZmqSubscriberBytes&) = delete;
  ZmqSubscriberBytes& operator=(const ZmqSubscriberBytes&) = delete;

  ZmqSubscriberBytes(ZmqSubscriberBytes&&) = delete;
  ZmqSubscriberBytes& operator=(ZmqSubscriberBytes&&) = delete;

  /**
   * @brief 수신/디스패치 루프에 stop 요청을 보낸다.
   */
  void Stop() noexcept;

  /**
   * @brief 현재 endpoint 값을 반환한다.
   *
   * @return endpoint 문자열 참조.
   */
  const std::string& Endpoint() const noexcept;

  /**
   * @brief 현재 topic 값을 반환한다.
   *
   * @return topic 문자열 참조.
   */
  const std::string& Topic() const noexcept;

 private:
  static constexpr std::string_view kClassName = "ZmqSubscriberBytes";

  void StartThreadsOrThrow();
  void RecvLoop() noexcept;
  void DispatchLoop() noexcept;

  void EnqueueOrDropOldest(std::vector<std::byte>&& payload);
  bool ShouldContinueOnError(const std::string& error_message) noexcept;

  void LogErrorPrefix(std::string_view message) const noexcept;
  void LogZmqErrorPrefix(std::string_view message, const zmq::error_t& error) const noexcept;

  static int CurrentPid() noexcept;

  zmq::context_t& context_;  ///< 외부에서 소유되는 ZMQ 컨텍스트 참조.
  zmq::socket_t socket_;     ///< SUB 소켓.
  std::string endpoint_;     ///< SUB 소켓 연결 endpoint.
  std::string topic_;        ///< 구독 토픽.
  std::size_t max_queue_size_;  ///< 내부 큐 최대 크기.
  MessageCallback on_message_;  ///< payload 수신 콜백.
  ErrorCallback on_error_;      ///< 오류 처리 콜백.
  std::atomic<bool> stop_requested_; ///< 종료 요청 플래그.

  std::mutex queue_mutex_;                   ///< 큐 보호 뮤텍스.
  std::condition_variable queue_condition_;  ///< 큐 소비자 깨움 조건 변수.
  std::deque<std::vector<std::byte>> queue_; ///< payload 큐.

  std::thread recv_thread_;     ///< SUB 수신 스레드.
  std::thread dispatch_thread_; ///< 콜백 디스패치 스레드.
};

}  // namespace zmq_pub

#endif  // ZMQ_SUBSCRIBER_BYTES_H_
