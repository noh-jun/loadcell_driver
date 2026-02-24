#include "loadcell_controller_node.h"

#include "data_builder.h"
#include "zmq_publisher_msgpack.hpp"

namespace loadcell_controller {
LoadcellControllerNode::LoadcellControllerNode() : rclcpp::Node("loadcell_controller_node"), zmq_context_(1) {
  // ---- Parameters ----
  topic_name_ = this->declare_parameter<std::string>("topic_name", "loadcell/meta");
  zmq_endpoint_ = this->declare_parameter<std::string>("zmq_endpoint", "tcp://*:5558");
  zmq_topic_ = this->declare_parameter<std::string>("zmq_topic", "loadcell");

  // ---- ROS2 subscriber ----
  subscription_ = this->create_subscription<MetaMsg>(topic_name_, rclcpp::QoS(10), std::bind(&LoadcellControllerNode::OnRosMessage, this, std::placeholders::_1));

  // ---- ZMQ publisher (MsgPackPublisher<DataResult>) ----
  publisher_ = std::make_unique<zmq_pub::MsgPackPublisher<LOADCELL::DataResult>>(zmq_context_, zmq_endpoint_, zmq_topic_);

  // ---- Builder ----
  data_builder_ = std::make_unique<DataBuilder>();
}

LoadcellControllerNode::~LoadcellControllerNode() = default;

void LoadcellControllerNode::OnRosMessage(const MetaMsg::SharedPtr message) {
  LOADCELL::DataResult data_result = data_builder_->Build(*message);

  try {
    publisher_->Publish(data_result);
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Error publishing to ZMQ: %s", e.what());
  }
  catch (...) {
    RCLCPP_ERROR(this->get_logger(), "Unknown error publishing to ZMQ");
  }
}

}  // namespace battery_driver
