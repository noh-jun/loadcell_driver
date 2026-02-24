#include "data_builder.h"
#include <iomanip>
#include <iostream>
#include <sstream>
#include <utility>

namespace loadcell_controller {
DataBuilder::DataBuilder() = default;

DataBuilder::~DataBuilder() = default;

LOADCELL::DataResult DataBuilder::Build(const MetaMsg& message) {
  LOADCELL::DataResult data_result;
  data_result.driver_instance_id = message.meta.driver_instance_id;
  data_result.seq_no = message.meta.seq_no;
  data_result.pub_timestamp = time_to_uint64(message.meta.pub_timestamp);
  data_result.driver_state = message.meta.driver_state;
  data_result.ret_code = message.meta.ret_code;
  data_result.driver_err_msg = message.meta.driver_err_msg;

  // LoadcellStatus 매핑
  data_result.status = LOADCELL::LoadcellStatus{};

  // Weight values
  data_result.status.gross_weight = message.values.gross_weight;
  data_result.status.right_weight = message.values.right_weight;
  data_result.status.left_weight = message.values.left_weight;

  // Right load cell battery/status
  data_result.status.right_battery_percent = message.values.right_battery_percent;
  data_result.status.right_charge_status = message.values.right_charge_status;
  data_result.status.right_online_status = message.values.right_online_status;

  // Left load cell battery/status
  data_result.status.left_battery_percent = message.values.left_battery_percent;
  data_result.status.left_charge_status = message.values.left_charge_status;
  data_result.status.left_online_status = message.values.left_online_status;

  // Measurement state flags
  data_result.status.gross_net_mark = message.values.gross_net_mark;
  data_result.status.overload_mark = message.values.overload_mark;
  data_result.status.out_of_tolerance_mark = message.values.out_of_tolerance_mark;

  PrintDebug(message);
  PrintDebug(data_result);

  return data_result;
}

void DataBuilder::PrintDebug(const MetaMsg& message) const noexcept {
  std::ostringstream oss;
  oss << "====== DataBuilder: Input MetaMsg ======\n";
  oss << "[Meta]\n";
  oss << "driver_instance_id  : " << message.meta.driver_instance_id << "\n";
  oss << "seq_no              : " << message.meta.seq_no << "\n";
  oss << "pub_timestamp       : " << time_to_string(message.meta.pub_timestamp) << "\n";
  oss << "driver_state        : " << state_to_string(message.meta.driver_state) << "\n";
  oss << "ret_code            : " << message.meta.ret_code << "\n";
  oss << "driver_err_msg      : " << message.meta.driver_err_msg << "\n";

  oss << "[Values]\n";
  oss << "gross_weight             : " << message.values.gross_weight << "\n";
  oss << "right_weight             : " << message.values.right_weight << "\n";
  oss << "left_weight              : " << message.values.left_weight << "\n";
  oss << "right_battery_percent    : " << static_cast<int>(message.values.right_battery_percent) << "\n";
  oss << "right_charge_status      : " << static_cast<int>(message.values.right_charge_status) << "\n";
  oss << "right_online_status      : " << static_cast<int>(message.values.right_online_status) << "\n";
  oss << "left_battery_percent     : " << static_cast<int>(message.values.left_battery_percent) << "\n";
  oss << "left_charge_status       : " << static_cast<int>(message.values.left_charge_status) << "\n";
  oss << "left_online_status       : " << static_cast<int>(message.values.left_online_status) << "\n";
  oss << "gross_net_mark           : " << static_cast<int>(message.values.gross_net_mark) << "\n";
  oss << "overload_mark            : " << static_cast<int>(message.values.overload_mark) << "\n";
  oss << "out_of_tolerance_mark    : " << static_cast<int>(message.values.out_of_tolerance_mark) << "\n";
  oss << "\n";


  std::cout << oss.str();
}

void DataBuilder::PrintDebug(const LOADCELL::DataResult& data_result) const noexcept {
  std::ostringstream oss;
  oss << "====== DataBuilder: Output DataResult ======\n";
  oss << "[DataResult]\n";
  oss << "driver_instance_id  : " << data_result.driver_instance_id << "\n";
  oss << "seq_no              : " << data_result.seq_no << "\n";
  oss << "pub_timestamp       : " << time_to_string(data_result.pub_timestamp) << "\n";
  oss << "driver_state        : " << state_to_string(data_result.driver_state) << "\n";
  oss << "ret_code            : " << data_result.ret_code << "\n";
  oss << "driver_err_msg      : " << data_result.driver_err_msg << "\n";

  oss << "[LoadcellStatus]\n";
  oss << "gross_weight             : " << data_result.status.gross_weight << "\n";
  oss << "right_weight             : " << data_result.status.right_weight << "\n";
  oss << "left_weight              : " << data_result.status.left_weight << "\n";
  oss << "right_battery_percent    : " << static_cast<int>(data_result.status.right_battery_percent) << "\n";
  oss << "right_charge_status      : " << static_cast<int>(data_result.status.right_charge_status) << "\n";
  oss << "right_online_status      : " << static_cast<int>(data_result.status.right_online_status) << "\n";
  oss << "left_battery_percent     : " << static_cast<int>(data_result.status.left_battery_percent) << "\n";
  oss << "left_charge_status       : " << static_cast<int>(data_result.status.left_charge_status) << "\n";
  oss << "left_online_status       : " << static_cast<int>(data_result.status.left_online_status) << "\n";
  oss << "gross_net_mark           : " << static_cast<int>(data_result.status.gross_net_mark) << "\n";
  oss << "overload_mark            : " << static_cast<int>(data_result.status.overload_mark) << "\n";
  oss << "out_of_tolerance_mark    : " << static_cast<int>(data_result.status.out_of_tolerance_mark) << "\n";
  oss << "\n";

  std::cout << oss.str();
}

std::uint64_t DataBuilder::time_to_uint64(const builtin_interfaces::msg::Time& t) const noexcept {
  return static_cast<std::uint64_t>(t.sec) * 1000000000ULL + static_cast<std::uint64_t>(t.nanosec);
}

std::string DataBuilder::time_to_string(const builtin_interfaces::msg::Time& t) const noexcept {
  using namespace std::chrono;

  // sec + nanosec → time_point
  system_clock::time_point tp = system_clock::time_point{seconds(t.sec) + nanoseconds(t.nanosec)};

  std::time_t tt = system_clock::to_time_t(tp);
  std::tm tm{};
  localtime_r(&tt, &tm);  // thread-safe

  std::ostringstream oss;
  oss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S");

  // millisecond
  auto ms = duration_cast<milliseconds>(tp.time_since_epoch()) % 1000;
  oss << "." << std::setw(3) << std::setfill('0') << ms.count();

  return oss.str();
}

std::string DataBuilder::time_to_string(std::uint64_t t) const noexcept {
  using namespace std::chrono;

  system_clock::time_point tp = system_clock::time_point{nanoseconds(t)};

  std::time_t tt = system_clock::to_time_t(tp);
  std::tm tm{};
  localtime_r(&tt, &tm);  // thread-safe

  std::ostringstream oss;
  oss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S");

  auto ms = duration_cast<milliseconds>(tp.time_since_epoch()) % 1000;
  oss << "." << std::setw(3) << std::setfill('0') << ms.count();

  return oss.str();
}

std::string DataBuilder::state_to_string(std::uint8_t state) const noexcept {
  switch (state) {
    case LOADCELL::DRIVER_STATE::OK:
      return "DRIVER_STATE::OK";
    case LOADCELL::DRIVER_STATE::EMPTY:
      return "DRIVER_STATE::EMPTY";
    case LOADCELL::DRIVER_STATE::ERROR:
      return "DRIVER_STATE::ERROR";
    default:
      return "UNKNOWN driver_state: " + std::to_string(state);
  }
}

}  // namespace loadcell_controller
