#include "data_builder.h"

namespace sensor_driver_base {
/**
* @brief DataBuilder 생성자.
*
* @param sensor_id 센서 식별자(파라미터로부터)
* @param driver_instance_id 드라이버 인스턴스 식별자(현재 프로세스의 드라이버임을 식별하기 위한 인자)
*/
DataBuilder::DataBuilder(std::string sensor_id, std::string driver_instance_id)
    : sensor_id_(std::move(sensor_id)), driver_instance_id_(std::move(driver_instance_id)) {}

/** @brief 기본 소멸자 */
DataBuilder::~DataBuilder() = default;

/**
* @brief DriverMeta 메시지를 생성합니다.
*
* @param seq_no 퍼블리시 시퀀스 번호
* @param pub_timestamp 퍼블리시 시각(ROS time)
* @param data_result 드라이버 수신 결과(상태/코드/에러 메시지 등)
* @return 구성된 DriverMeta 메시지
*
* @note payload는 센서별 구현에서 확장해야 하며 현재는 비워둡니다.
*/
MetaMsg DataBuilder::Build(uint64_t seq_no, const rclcpp::Time& pub_timestamp, const DataResult& data_result) {
  MetaMsg msg;
  msg.meta.sensor_id = sensor_id_;
  msg.meta.driver_instance_id = driver_instance_id_;
  msg.meta.seq_no = seq_no;
  msg.meta.pub_timestamp = pub_timestamp;
  msg.meta.driver_state = data_result.driver_state;
  msg.meta.ret_code = data_result.return_code;
  msg.meta.driver_err_msg = data_result.driver_err_msg;
  // payload 는 sensor 구현 시 추가 필요
  // payload: loadcell
  msg.values.gross_weight = data_result.status.gross_weight;
  msg.values.right_weight = data_result.status.right_weight;
  msg.values.left_weight = data_result.status.left_weight;

  msg.values.right_battery_percent = data_result.status.right_battery_percent;
  msg.values.right_charge_status = data_result.status.right_charge_status;
  msg.values.right_online_status = data_result.status.right_online_status;

  msg.values.left_battery_percent = data_result.status.left_battery_percent;
  msg.values.left_charge_status = data_result.status.left_charge_status;
  msg.values.left_online_status = data_result.status.left_online_status;

  msg.values.gross_net_mark = data_result.status.gross_net_mark;
  msg.values.overload_mark = data_result.status.overload_mark;
  msg.values.out_of_tolerance_mark = data_result.status.out_of_tolerance_mark;

  return msg;
}
}  // namespace sensor_driver_base