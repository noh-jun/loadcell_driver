#pragma once

#include <rclcpp/rclcpp.hpp>

#include "data_result.h"
#include <sensor_driver_interface/msg/weight_meta.hpp>

/** 메시지 인터페이스 alias */
using MetaMsg = sensor_driver_interface::msg::WeightMeta;

namespace sensor_driver_base {
class DataBuilder {
public:
  /**
   * @brief DataBuilder 생성자.
   *
   * @param sensor_id 센서 식별자(파라미터로부터)
   * @param driver_instance_id 드라이버 인스턴스 식별자(현재 프로세스의
   * 드라이버임을 식별하기 위한 인자)
   */
  DataBuilder(std::string sensor_id, std::string driver_instance_id);
  ~DataBuilder();

  MetaMsg Build(uint64_t seq_no, const rclcpp::Time &pub_timestamp, const DataResult &data_result);

private:
  /** @brief 센서 식별자 */
  std::string sensor_id_;
  /** @brief 드라이버 인스턴스 식별자 */
  std::string driver_instance_id_;
};

} // namespace sensor_driver_base