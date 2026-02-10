// loadcell_driver_node.cpp
#include "loadcell_driver_node.h"

#include <loadcell_comm/SerialConfig.h>
#include <loadcell_comm/loadcell_485.h>
#include <loadcell_comm/loadcell_exception.h>

namespace loadcell_sensor {

LoadCellDriverNode::LoadCellDriverNode()
    : loadcell_(std::make_unique<loadcell_comm::LoadCell485>()) {}

LoadCellDriverNode::~LoadCellDriverNode() = default;

void LoadCellDriverNode::ParamChange() {
  loadcell_port_ = this->declare_parameter<std::string>("loadcell_port", "none");
  // 485 통신인데 address 를 사용하지 않음
  // loadcell_address_ = this->declare_parameter<int>("loadcell_address", -1);
  loadcell_baudrate_ = this->declare_parameter<int>("loadcell_baudrate", 9600);
  loadcell_databits_ = this->declare_parameter<int>("loadcell_databits", 8);
  loadcell_parity_ = this->declare_parameter<std::string>("loadcell_parity", "N");
  loadcell_stopbits_ = this->declare_parameter<int>("loadcell_stopbits", 1);
}

bool LoadCellDriverNode::IsConnected() {
  if (!loadcell_)
    return false;

  return loadcell_->IsOpen();
}

void LoadCellDriverNode::Connect() {
  SerialConfig cfg;
  cfg.device = loadcell_port_;
  cfg.baudrate = loadcell_baudrate_;
  cfg.data_bits = loadcell_databits_;
  cfg.parity = loadcell_parity_[0];  // "N" -> 'N'
  cfg.stop_bits = loadcell_stopbits_;
  // 기타 설정은 기본값 사용

  const bool ok_open = loadcell_->Open(cfg);
  if (!ok_open) {
    std::ostringstream oss;
    oss << "LoadCellDriverNode: BatteryTabos485::Open() failed. ";
    oss << "Device: [ " << cfg.device << "] ";
    oss << "last_error=" << loadcell_->GetLastError();
    throw std::runtime_error(oss.str());
  }
}

void LoadCellDriverNode::GetData(sensor_driver_base::DataResult& out) {
  try {
    loadcell_comm::LoadCellStatus status;
    const loadcell_comm::ResultCode Res = loadcell_->RecvOnce(status);

    switch (Res) {
      case loadcell_comm::ResultCode::kOk:
        out.driver_state = sensor_driver_base::DRIVER_STATE::OK;
        out.return_code = static_cast<std::int32_t>(Res);
        out.driver_err_msg = "";
        out.status = std::move(status);
        break;
      case loadcell_comm::ResultCode::kFrameTooShort:
      case loadcell_comm::ResultCode::kNoFrame:
        out.driver_state = sensor_driver_base::DRIVER_STATE::EMPTY;
        out.return_code = static_cast<std::int32_t>(Res);
        out.driver_err_msg = loadcell_->GetLastError();
        break;
      case loadcell_comm::ResultCode::kIoReadFail:
      default:
        out.driver_state = sensor_driver_base::DRIVER_STATE::ERROR;
        out.return_code = static_cast<std::int32_t>(Res);
        out.driver_err_msg = loadcell_->GetLastError();
        out.status = std::move(status);
        break;
    }
  }
  catch (const loadcell_comm::LoadCell485Exception& ex) {
    std::ostringstream oss;
    oss << "LoadCellDriverNode::GetData: LoadCell485Exception. ";
    oss << "code=" << ex.Code() << " ";
    oss << "what=" << ex.what();

    out.driver_state = sensor_driver_base::DRIVER_STATE::ERROR;
    out.return_code = std::numeric_limits<std::int32_t>::min();
    out.driver_err_msg = oss.str();
    out.status = loadcell_comm::LoadCellStatus{};

    throw std::runtime_error(oss.str());
  }
}

void LoadCellDriverNode::PrintPublishData(MetaMsg& msg) {
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

  // ROS 로거에 붙이고 싶으면 SensorDriverNode가 제공하는 logger API를 사용하십시오. 여기서는 표준 출력으로 처리합니다.
  std::cout << oss.str() << std::endl;
}

}  // namespace loadcell_sensor
