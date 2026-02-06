#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "data_builder.h"

/** 전방선언 */
namespace sensor_driver_uk {
class GenSeqNo;
}

namespace sensor_driver_base {

/**
 * @brief 센서 드라이버의 IO 처리 및 메타 정보 퍼블리시를 담당하는 ROS2 노드.
 *
 * 두 개의 버퍼(더블 버퍼)를 사용하여
 * - IO 스레드에서 수신한 데이터를 write 슬롯에 채운 뒤
 * - 짧은 락 구간에서 read/write 인덱스를 스왑하고,
 * - 퍼블리시 스레드는 read 슬롯을 스냅샷으로 읽어 publish 합니다.
 *
 * @note payload(실제 센서 데이터)는 sensor 구현 시 확장해야 하며, 현재는 DriverMeta 메시지의 기본 메타 필드 위주로 publish 합니다.
 */
class SensorDriverNode : public rclcpp::Node {
 public:
  /**
   * @brief SensorDriverNode 생성자.
   * 파라미터 선언(센서 ID, 토픽명 등), 시퀀스 생성기/빌더 생성, publisher/thread 설정 및 버퍼 초기화를 수행합니다.
   */
  SensorDriverNode();
  SensorDriverNode(const std::string& node_name);

  /**
   * @brief SensorDriverNode 소멸자.
   * Thread 종료 및 자원 해제를 수행합니다. (RAII)
   */
  ~SensorDriverNode();

  /**
   * @brief 초기화
   */
  void Initialize();

 private:
  /**
   * @brief 센서 드라이버 IO 주기 처리 콜백.
   *
   * - 드라이버 연결 상태 확인/재연결(향후 TODO)
   * - 드라이버 옵션 변경(향후 TODO)
   * - 드라이버 데이터 수신 후 write 버퍼에 저장
   * - 짧은 락 구간에서 read/write 인덱스를 스왑하여 최신 데이터 공개
   *
   * @note 현재 구현은 예시(placeholder)이며 실제 수신 로직은 TODO 입니다.
   */
  void IO_Callback();

  /**
   * @brief 퍼블리시 주기 처리 콜백.
   * 연결 상태가 정상이고 최신 데이터가 준비되었으면(has_data_가 true) read 슬롯 스냅샷을 사용해 PublishOnce()를 호출합니다.
   */
  void PublisherCallback();

  /**
   * @brief 지정한 read 슬롯의 데이터를 1회 publish 합니다.
   * @param read_index publish에 사용할 read 버퍼 인덱스(0 또는 1).
   * @note publish 이후 시퀀스 번호를 증가시킵니다.
   */
  void PublishOnce(std::size_t read_index);

  void IoThreadMain();
  void PublishThreadMain();

  /** @brief 기본 파라미터를 변경 시 상속하여 재선언 */
  virtual void ParamChange() {};
  /** @brief 연결 확인 필요 시 상속하여 재선언 */
  virtual bool IsConnected() { return true; };
  /** @brief 연결 수행 필요 시 상속하여 재선언 */
  virtual void Connect() {};
  /** @brief Driver 파라미터 변경 여부 확인 필요 시 상속하여 재선언*/
  virtual bool IsChangedOption() { return false; };
  /** @brief Driver 파라미터 변경 필요 시 상속하여 재선언 */
  virtual void ChangeOption() {};
  /** @brief Driver 에서 실제 data 를 읽을 경우 상속하여 재선언 */
  virtual void GetData(DataResult&) {};
  /** @brief Publish Data 를 확인하고 싶다면 상속하여 재선언 */
  virtual void PrintPublishData(MetaMsg&);

 protected:
  /** @brief publish 메시지의 seq_no 생성을 위한 유니크 키(시퀀스) 생성기 */
  std::unique_ptr<sensor_driver_uk::GenSeqNo> gen_seq_no_;

  /** @brief DataResult -> DriverMeta 메시지 변환을 담당하는 빌더 */
  std::unique_ptr<DataBuilder> data_builder_;

  /** @brief DriverMeta 메시지 퍼블리셔 */
  rclcpp::Publisher<MetaMsg>::SharedPtr publisher_;

  /** @brief IO 처리용 타이머(예: 500ms) */
  rclcpp::TimerBase::SharedPtr timer_io_;

  /** @brief 퍼블리시용 타이머(100ms 고정) */
  rclcpp::TimerBase::SharedPtr timer_publish_;

  /**
   * @brief 드라이버 연결 상태 플래그.
   * true: 정상 연결/동작 가능, false: 미연결(재연결 필요)
   */
  std::atomic<bool> connection_state_{false};

  /**
   * @brief 더블 버퍼: 최신 수신 데이터를 저장하는 버퍼.
   * read_index_가 가리키는 슬롯은 publisher가 읽고,
   * write_index_가 가리키는 슬롯은 IO가 씁니다.
   */
  std::array<DataResult, 2> data_buffer_{};

  /** @brief 현재 publish 대상(read) 버퍼 인덱스(0/1) */
  int read_index_{0};

  /** @brief 현재 IO 쓰기 대상(write) 버퍼 인덱스(0/1) */
  int write_index_{1};

  /**
   * @brief 최초 데이터 수신 여부.
   * true가 되기 전에는 publish 정책에 따라 스킵하거나 EMPTY 상태를 내보낼 수 있습니다.
   */
  bool has_data_{false};

  /**
   * @brief read/write 인덱스 스왑 및 has_data_ 갱신을 보호하는 뮤텍스.
   * @note 데이터 버퍼 전체를 장시간 잠그지 않고, 인덱스 교체 구간만 짧게 보호합니다.
   */
  std::mutex data_mutex_;

  /** @brief  */
  std::chrono::milliseconds io_period_{500};
  /** @brief  */
  std::chrono::milliseconds publish_period_{100};

  /** @brief  */
  std::atomic<bool> stop_requested_{false};
  /** @brief  */
  std::thread io_thread_;
  /** @brief  */
  std::thread publish_thread_;
};

}  // namespace sensor_driver_base