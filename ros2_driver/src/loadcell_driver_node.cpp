// loadcell_driver_node.cpp

#include "loadcell_driver_node.h"

#include <cstdint>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <loadcell_comm/rs485/loadcell_comm/loadcell_485.hpp>
#include <loadcell_comm/rs485/loadcell_comm/SerialConfig.h>

namespace sensor_driver_base {

LoadCellDriverNode::LoadCellDriverNode()
    : loadcell_(std::make_unique<LoadCell485>()) {}

LoadCellDriverNode::~LoadCellDriverNode() = default;

void LoadCellDriverNode::ParamChange() {
  loadcell_port_ = this->declare_parameter<std::string>("loadcell_port", "none");
  loadcell_address_ = this->declare_parameter<int>("loadcell_address", -1);
}

bool LoadCellDriverNode::IsConnected() {
  if (!loadcell_)
    return false;

  return loadcell_->IsOpen();
}

void LoadCellDriverNode::Connect() {
  if (!loadcell_)
    throw std::runtime_error("LoadCellDriverNode: loadcell_ is null");

  SerialConfig cfg;
  cfg.device = loadcell_port_;
  cfg.baudrate = 9600;
  cfg.data_bits = 8;
  cfg.parity = 'N';
  cfg.stop_bits = 1;
  cfg.rtscts = false;
  cfg.xonxoff = false;
  cfg.vmin = 0;
  cfg.vtime_ds = 1;
  const bool ok_open = loadcell_->Open(cfg);
  if (!ok_open) {
    std::ostringstream oss;
    oss << "LoadCellDriverNode: BatteryTabos485::Open() failed. ";
    oss << "Device: [ " << cfg.device << "] ";
    oss << "last_error=\"" << loadcell_->LastError() << "\" ";
    ;
    oss << "io_error=\"" << loadcell_->LastIoError() << "\" ";
    throw std::runtime_error(oss.str());
  }
}

void LoadCellDriverNode::GetData(DataResult &out) {
  if (!loadcell_) {
    throw std::runtime_error("LoadCellDriverNode::GetData: loadcell_ is null");
  }

  // LoadCell 프로토콜이 요청-응답형이면 polling 프레임을 먼저 보낸다.
  // 스트리밍형이면 SendPoll() 호출을 제거하거나 파라미터로 토글하십시오.
  const int send_rc = loadcell_->SendPoll();
  if (LoadCell485::ResultCode::kOk != send_rc) {
    std::ostringstream oss;
    oss << "LoadCellDriverNode::GetData: SendPoll failed. ";
    oss << "Result Code=" << send_rc << " ";
    oss << "last_error=\"" << loadcell_->LastError() << "\" ";
    oss << "io_error=\"" << loadcell_->LastIoError() << "\"";
    throw std::runtime_error(oss.str());
  }

  try {
    while (true) {
      const int ret = loadcell_->RecvOnce(out.status);

      if (LoadCell485::ResultCode::kNoFrame == ret) {
        // 예제의 의도대로 non-blocking retry.
        // CPU busy-spin이 문제가 되면 sleep/backoff 또는 timeout을 도입하십시오.
        continue;
      } else if (LoadCell485::ResultCode::kOk == ret) {
        out.driver_state = DRIVER_STATE::OK;
        out.return_code = ret;
        out.driver_err_msg = "";
        break;
      } else {
        std::ostringstream oss;
        oss << "LoadCellDriverNode::GetData: RecvOnce failed. ";
        oss << "Return Code=" << ret << " ";
        oss << "last_error=\"" << loadcell_->LastError() << "\" ";
        oss << "io_error=\"" << loadcell_->LastIoError() << "\"";

        std::ostringstream error;
        error << loadcell_->LastError() << "/" << loadcell_->LastIoError();

        out.driver_state = DRIVER_STATE::ERROR;
        out.return_code = ret;
        out.driver_err_msg = error.str();
        out.status = LoadCellStatus{};

        throw std::runtime_error(oss.str());
      }
    }
  } catch (const LoadCell485Exception &ex) {
    std::ostringstream oss;
    oss << "LoadCellDriverNode::GetData: LoadCell485Exception. ";
    oss << "code=" << ex.Code() << " ";
    oss << "what=\"" << ex.what() << "\"";

    out.driver_state = DRIVER_STATE::ERROR;
    out.return_code = std::numeric_limits<std::int32_t>::min();
    out.driver_err_msg = oss.str();
    out.status = LoadCellStatus{};

    throw std::runtime_error(oss.str());
  }
}

void LoadCellDriverNode::PrintPublishData(MetaMsg &msg) {
  std::ostringstream oss;
  oss << "=== LoadCellMeta ===\n";
  oss << "gross_weight=" << msg.values.gross_weight << "\n";
  oss << "right_weight=" << msg.values.right_weight << "\n";
  oss << "left_weight=" << msg.values.left_weight << "\n";

  oss << "right_battery_percent=" << static_cast<int>(msg.values.right_battery_percent) << "\n";
  oss << "right_charge_status=" << static_cast<int>(msg.values.right_charge_status) << "\n";
  oss << "right_online_status=" << static_cast<int>(msg.values.right_online_status) << "\n";

  oss << "left_battery_percent=" << static_cast<int>(msg.values.left_battery_percent) << "\n";
  oss << "left_charge_status=" << static_cast<int>(msg.values.left_charge_status) << "\n";
  oss << "left_online_status=" << static_cast<int>(msg.values.left_online_status) << "\n";

  oss << "gross_net_mark=" << static_cast<int>(msg.values.gross_net_mark) << "\n";
  oss << "overload_mark=" << static_cast<int>(msg.values.overload_mark) << "\n";
  oss << "out_of_tolerance_mark=" << static_cast<int>(msg.values.out_of_tolerance_mark) << "\n";

  // ROS 로거에 붙이고 싶으면 SensorDriverNode가 제공하는 logger API를
  // 사용하십시오. 여기서는 표준 출력으로 처리합니다.
  std::cout << oss.str() << std::endl;
}

} // namespace sensor_driver_base
