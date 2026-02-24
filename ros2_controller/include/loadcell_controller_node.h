#ifndef LOADCELL_CONTROLLER_NODE_H
#define LOADCELL_CONTROLLER_NODE_H

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <zmq.hpp>
#include <sensor_driver_interface/msg/weight_meta.hpp>

#include "zmq_data_type.h"

namespace zmq_pub {
template <typename T> class MsgPackPublisher;
} // namespace zmq_pub

namespace loadcell_controller {
class DataBuilder;
}

using MetaMsg = sensor_driver_interface::msg::WeightMeta;

namespace loadcell_controller {
/**
 * @brief ROS2 MetaMsg 메시지를 수신해 LOADCELL::DataResult로 변환한 뒤 ZMQ로 발행하는 노드.
 *
 * MetaMsg 토픽을 구독하고, 수신 콜백에서 DataBuilder로 LOADCELL::DataResult를 구성한다.
 * 구성된 결과는 MsgPackPublisher<LOADCELL::DataResult>를 통해 MsgPack 직렬화 후 endpoint/topic으로 발행한다.
 *
 * @note ZMQ 컨텍스트는 노드가 소유하며, publisher_는 컨텍스트보다 먼저  파괴되도록 멤버 순서를 유지한다.
 */
class LoadcellControllerNode final : public rclcpp::Node {
public:
  /**
   * @brief 파라미터를 선언/조회하고 ROS2 구독자와 ZMQ 퍼블리셔를 초기화한다.
   */
  LoadcellControllerNode();

  /**
   * @brief ROS 구독과 ZMQ 퍼블리셔 리소스를 정리한다.
   */
  ~LoadcellControllerNode() override;

private:
  /**
   * @brief MetaMsg 수신 콜백에서 DataResult를 구성하고 ZMQ로 발행한다.
   *
   * @param message 수신된 MetaMsg 메시지.
   */
  void OnRosMessage(const MetaMsg::SharedPtr message);

  std::string topic_name_;   ///< 구독할 ROS2 토픽 이름.
  std::string zmq_endpoint_; ///< ZMQ PUB endpoint.
  std::string zmq_topic_;    ///< ZMQ 발행 토픽.

  rclcpp::Subscription<MetaMsg>::SharedPtr subscription_; ///< ROS2 구독자.

  zmq::context_t zmq_context_; ///< ZMQ 컨텍스트(노드가 소유).

  std::unique_ptr<zmq_pub::MsgPackPublisher<LOADCELL::DataResult>> publisher_; ///< DataResult MsgPack 퍼블리셔.
  std::unique_ptr<DataBuilder> data_builder_; ///< ROS 메시지를 DataResult로 변환하는 빌더.
};

} // namespace battery_driver

#endif // LOADCELL_CONTROLLER_NODE_H
