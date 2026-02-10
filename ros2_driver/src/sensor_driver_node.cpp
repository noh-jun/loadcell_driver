#include "sensor_driver_node.h"

#include <chrono>
#include <exception>
#include <sstream>
#include <utility>

#include "unique_key_generator.h"

using namespace std::chrono_literals;

namespace sensor_driver_base {
/**
 * @brief SensorDriverNode 생성자.
 *
 * - ROS 파라미터(sensor_id, topic_name) 선언
 * - driver_instance_id 생성
 * - seq generator / data builder 초기화
 * - publisher 생성
 * - IO 타이머(500ms), publish 타이머(100ms) 설정
 * - 버퍼 초기값 세팅(최초 publish 전에 쓰레기값 방지)
 */
SensorDriverNode::SensorDriverNode() : rclcpp::Node("sensor_driver_node") {}

SensorDriverNode::SensorDriverNode(const std::string& node_name)
    : rclcpp::Node(node_name) {}

SensorDriverNode::~SensorDriverNode() {
  /** thread stop flag 활성화 */
  stop_requested_.store(true, std::memory_order_relaxed);

  /** Driver IO thread 종료 대기 */
  if (io_thread_.joinable())
    io_thread_.join();

  /** Driver Data Publish thread 종료 대기 */
  if (publish_thread_.joinable())
    publish_thread_.join();
}

void SensorDriverNode::Initialize() {
  /** Parameters : 센서 식별자 선언 */
  const std::string sensor_id = declare_parameter<std::string>("sensor_id", "example_sensor");
  /** Parameters : 퍼블리시 토픽 이름 선언 */
  const std::string topic_name = declare_parameter<std::string>("topic_name", "driver_meta");

  /** Unique Key Generator */
  gen_seq_no_ = std::make_unique<sensor_driver_uk::GenSeqNo>();
  /** Parameters : 드라이버 인스턴스 식별자 선언 */
  const std::string driver_instance_id = sensor_driver_uk::GenerateInstanceID(sensor_id);
  /** Data Builder */
  data_builder_ = std::make_unique<DataBuilder>(sensor_id, driver_instance_id);

  /** Publisher */
  publisher_ = this->create_publisher<MetaMsg>(topic_name, 1);

  try {
    /** user params 변경 사항을 반영하기 위해 호출 */
    ParamChange();

    /** user 연결 구현 시 자동 연결을 위해 호출 */
    Connect();
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    // Not process
  }

  /** thread stop flag 초기화*/
  stop_requested_.store(false, std::memory_order_relaxed);
  /** Driver IO thread 생성 */
  io_thread_ = std::thread(&SensorDriverNode::IoThreadMain, this);
  /** Driver Data Publish thrad 생성 */
  publish_thread_ = std::thread(&SensorDriverNode::PublishThreadMain, this);
}

/**
 * @brief
 * @note
 */
void SensorDriverNode::IoThreadMain() {
  auto next_wakeup = std::chrono::steady_clock::now();

  while (rclcpp::ok() && !stop_requested_.load(std::memory_order_relaxed)) {
    next_wakeup += io_period_;

    IO_Callback();

    std::this_thread::sleep_until(next_wakeup);
  }
}

/**
 * @brief
 * @note
 */
void SensorDriverNode::PublishThreadMain() {
  auto next_wakeup = std::chrono::steady_clock::now();

  while (rclcpp::ok() && !stop_requested_.load(std::memory_order_relaxed)) {
    next_wakeup += publish_period_;

    PublisherCallback();

    std::this_thread::sleep_until(next_wakeup);
  }
}

/**
 * @brief 드라이버 IO 처리 콜백(타이머 기반).
 *
 * 수행 흐름(현재는 TODO/placeholder 포함):
 * 1) 연결 상태 확인 및 재연결 시도(필요 시)
 * 2) 옵션 변경(필요 시)
 * 3) 데이터 수신 결과(DataResult)를 write 슬롯에 채움
 * 4) read/write 슬롯 인덱스를 스왑하여 최신 데이터 공개
 *
 * @note 스레드 안전:
 * - write 슬롯 인덱스 스냅샷 및 swap 구간만 짧게 mutex로 보호합니다.
 * - 실제 수신 및 데이터 채움은 lock 없이 수행하여 지연을 줄입니다.
 */
void SensorDriverNode::IO_Callback() {
  /** 드라이버 연결 상태 확인 */
  const bool state = IsConnected();

  /** 재연결 시도 */
  if (false == state) {
    try {
      Connect();

      /** 재연결 성공 */
      connection_state_.store(true, std::memory_order_relaxed);
    }
    catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "%s", e.what());

      connection_state_.store(false, std::memory_order_relaxed);

      /** 재연결 실패 시 다음 동작 Skip */
      return;
    }
  }
  else {
    // placeholder: 연결된 것으로 간주
    connection_state_.store(true, std::memory_order_relaxed);
  }

  try {
    /** 드라이버 옵션 변경 */
    if (true == IsChangedOption())
      ChangeOption();
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    // Not process
  }

  /** 드라이버 데이터 수신 */

  // write 슬롯 스냅샷: lock은 짧게
  std::size_t local_write_index = 0;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    local_write_index = write_index_;
  }

  try {
    // lock 없이 write 슬롯에 데이터 채우기
    GetData(data_buffer_[local_write_index]);

    // 채움 완료 후 swap: lock은 짧게
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      write_index_ = read_index_;
      read_index_ = local_write_index;
      has_data_ = true;
    }
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
  }
}

/**
 * @brief 퍼블리시 타이머 콜백.
 *
 * - 연결 실패 시 skip
 * - read 슬롯/has_data를 스냅샷으로 확보(짧은 lock)
 * - 최초 수신 전 정책에 따라 publish 스킵(현재 정책)
 * - 준비된 데이터가 있으면 PublishOnce() 호출
 */
void SensorDriverNode::PublisherCallback() {
  /** 드라이버 연결 실패 시 Publish Skip */
  if (false == connection_state_.load(std::memory_order_relaxed)) {
    return;
  }

  // read 슬롯 스냅샷: lock은 짧게
  std::size_t local_read_index = 0;
  bool has_data_snapshot = false;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    local_read_index = read_index_;
    has_data_snapshot = has_data_;
  }

  // 최초 수신 전이면 publish를 하지 않거나(정책), EMPTY 상태를 publish할 수
  // 있음. 여기서는 "최초 수신 전에는 publish 스킵" 정책으로 둠.
  if (!has_data_snapshot) {
    return;
  }

  PublishOnce(local_read_index);
}

/**
 * @brief 지정한 read 슬롯의 최신 데이터를 1회 publish 합니다.
 *
 * @param read_index publish 대상 버퍼 인덱스(스냅샷으로 전달된 값)
 *
 * @note publish 이후 시퀀스 번호를 증가시킵니다.
 */
void SensorDriverNode::PublishOnce(std::size_t read_index) {
  /** Message 생성 */
  MetaMsg msg = data_builder_->Build(gen_seq_no_->Current(), this->now(),
                                     data_buffer_[read_index]);

  PrintPublishData(msg);

  /** Message 퍼블리시 */
  publisher_->publish(msg);

  /** 시퀀스 번호 증가 */
  gen_seq_no_->Increment();
}

void SensorDriverNode::PrintPublishData(MetaMsg&) {
  // override
}
}  // namespace sensor_driver_base