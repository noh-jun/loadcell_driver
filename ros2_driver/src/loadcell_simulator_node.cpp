#include "loadcell_simulator_node.h"

#include <chrono>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <utility>

namespace loadcell_sensor {

LoadCellSimulatorNode::LoadCellSimulatorNode() {
  std::random_device random_device;
  rng_ = std::mt19937(random_device());
}

LoadCellSimulatorNode::~LoadCellSimulatorNode() = default;

void LoadCellSimulatorNode::ParamChange() {
  scenario_text_ = this->declare_parameter<std::string>("scenario", "normal");

  weight_min_kg_ = this->declare_parameter<double>("weight_min_kg", 0.0);
  weight_max_kg_ = this->declare_parameter<double>("weight_max_kg", 200.0);
  weight_noise_kg_ = this->declare_parameter<double>("weight_noise_kg", 1.0);

  const int battery_min = this->declare_parameter<int>("battery_min_percent", 20);
  const int battery_max = this->declare_parameter<int>("battery_max_percent", 100);

  battery_min_percent_ = static_cast<std::uint8_t>(std::max(0, std::min(100, battery_min)));
  battery_max_percent_ = static_cast<std::uint8_t>(std::max(0, std::min(100, battery_max)));
  if (battery_min_percent_ > battery_max_percent_) {
    std::swap(battery_min_percent_, battery_max_percent_);
  }
}

bool LoadCellSimulatorNode::IsConnected() {
  return connected_;
}

void LoadCellSimulatorNode::Connect() {
  // 시뮬레이터는 연결 절차가 없으므로 “연결됨”으로 간주
  connected_ = true;
}

void LoadCellSimulatorNode::GetData(sensor_driver_base::DataResult& out) {
  const Scenario scenario = ParseScenario(scenario_text_);

  // 기본값
  out.driver_state = sensor_driver_base::DRIVER_STATE::OK;
  out.return_code = 0;
  out.driver_err_msg.clear();

  loadcell_comm::LoadCellStatus status{};
  FillStatusByScenario(scenario, status);

  switch (scenario) {
    case Scenario::kNormal:
    case Scenario::kRightOffline:
    case Scenario::kLeftOffline:
    case Scenario::kRightHwFailure:
    case Scenario::kLeftHwFailure:
    case Scenario::kOverload:
    case Scenario::kOutOfToleranceLeft:
    case Scenario::kOutOfToleranceRight:
      out.driver_state = sensor_driver_base::DRIVER_STATE::OK;
      out.return_code = 0;
      out.driver_err_msg = "";
      out.status = std::move(status);
      break;

    case Scenario::kEmpty:
      out.driver_state = sensor_driver_base::DRIVER_STATE::EMPTY;
      out.return_code = 1;  // 임의 코드(드라이버의 ResultCode가 있다면 그 값에 맞추는 것이 최선)
      out.driver_err_msg = "simulator: empty frame";
      // EMPTY에서는 status를 넣지 않는 드라이버도 많으니 비워둠(필요하면 넣어도 됨)
      break;

    case Scenario::kError:
    default:
      out.driver_state = sensor_driver_base::DRIVER_STATE::ERROR;
      out.return_code = std::numeric_limits<std::int32_t>::min();
      out.driver_err_msg = "simulator: forced error scenario";
      out.status = std::move(status);
      break;
  }
}

void LoadCellSimulatorNode::PrintPublishData(MetaMsg& msg) {
  std::ostringstream output;
  output << "=== LoadCellMeta (SIMULATOR) ===\n";
  output << "gross_weight=" << msg.values.gross_weight << "\n";
  output << "right_weight=" << msg.values.right_weight << "\n";
  output << "left_weight=" << msg.values.left_weight << "\n";

  output << "right_battery_percent=" << static_cast<int>(msg.values.right_battery_percent) << "\n";
  output << "right_charge_status=" << static_cast<int>(msg.values.right_charge_status) << "\n";
  output << "right_online_status=" << static_cast<int>(msg.values.right_online_status) << "\n";

  output << "left_battery_percent=" << static_cast<int>(msg.values.left_battery_percent) << "\n";
  output << "left_charge_status=" << static_cast<int>(msg.values.left_charge_status) << "\n";
  output << "left_online_status=" << static_cast<int>(msg.values.left_online_status) << "\n";

  output << "gross_net_mark=" << static_cast<int>(msg.values.gross_net_mark) << "\n";
  output << "overload_mark=" << static_cast<int>(msg.values.overload_mark) << "\n";
  output << "out_of_tolerance_mark=" << static_cast<int>(msg.values.out_of_tolerance_mark) << "\n";

  std::cout << output.str() << std::endl;
}

LoadCellSimulatorNode::Scenario LoadCellSimulatorNode::ParseScenario(const std::string& scenario_text) const {
  if (scenario_text == "normal") return Scenario::kNormal;
  if (scenario_text == "empty") return Scenario::kEmpty;
  if (scenario_text == "error") return Scenario::kError;
  if (scenario_text == "right_offline") return Scenario::kRightOffline;
  if (scenario_text == "left_offline") return Scenario::kLeftOffline;
  if (scenario_text == "right_hw_failure") return Scenario::kRightHwFailure;
  if (scenario_text == "left_hw_failure") return Scenario::kLeftHwFailure;
  if (scenario_text == "overload") return Scenario::kOverload;
  if (scenario_text == "oot_left") return Scenario::kOutOfToleranceLeft;
  if (scenario_text == "oot_right") return Scenario::kOutOfToleranceRight;
  return Scenario::kNormal;
}

double LoadCellSimulatorNode::RandomDouble(double min_value, double max_value) {
  std::uniform_real_distribution<double> distribution(min_value, max_value);
  return distribution(rng_);
}

std::uint8_t LoadCellSimulatorNode::RandomU8(std::uint8_t min_value, std::uint8_t max_value) {
  std::uniform_int_distribution<int> distribution(static_cast<int>(min_value), static_cast<int>(max_value));
  return static_cast<std::uint8_t>(distribution(rng_));
}

void LoadCellSimulatorNode::FillStatusByScenario(Scenario scenario, loadcell_comm::LoadCellStatus& status) {
  // 기본: online / 정상 플래그
  status.right_online_status = 0;  // 0 online
  status.left_online_status = 0;   // 0 online
  status.right_charge_status = 0;
  status.left_charge_status = 0;

  status.right_battery_percent = RandomU8(battery_min_percent_, battery_max_percent_);
  status.left_battery_percent = RandomU8(battery_min_percent_, battery_max_percent_);

  status.gross_net_mark = 0;        // gross
  status.overload_mark = 0;
  status.out_of_tolerance_mark = 0; // ok

  // weights: left/right 생성 후 gross는 합으로 맞춤(일관성)
  const double base_right = RandomDouble(weight_min_kg_, weight_max_kg_);
  const double base_left = RandomDouble(weight_min_kg_, weight_max_kg_);

  const double noise_right = RandomDouble(-weight_noise_kg_, weight_noise_kg_);
  const double noise_left = RandomDouble(-weight_noise_kg_, weight_noise_kg_);

  status.right_weight = std::max(0.0, base_right + noise_right);
  status.left_weight = std::max(0.0, base_left + noise_left);
  status.gross_weight = status.right_weight + status.left_weight;

  // scenario 반영
  switch (scenario) {
    case Scenario::kNormal:
      break;

    case Scenario::kRightOffline:
      status.right_online_status = 1;  // offline
      break;

    case Scenario::kLeftOffline:
      status.left_online_status = 1;   // offline
      break;

    case Scenario::kRightHwFailure:
      status.right_online_status = 2;  // hw failure
      break;

    case Scenario::kLeftHwFailure:
      status.left_online_status = 2;   // hw failure
      break;

    case Scenario::kOverload:
      status.overload_mark = 1;
      // overload를 더 “그럴듯”하게: gross를 급증
      status.right_weight = std::max(status.right_weight, weight_max_kg_ * 1.2);
      status.left_weight = std::max(status.left_weight, weight_max_kg_ * 1.2);
      status.gross_weight = status.right_weight + status.left_weight;
      break;

    case Scenario::kOutOfToleranceLeft:
      status.out_of_tolerance_mark = 1; // left
      break;

    case Scenario::kOutOfToleranceRight:
      status.out_of_tolerance_mark = 2; // right
      break;

    case Scenario::kEmpty:
      // EMPTY는 상위 GetData에서 처리. 값 자체는 의미 없음.
      break;

    case Scenario::kError:
    default:
      // ERROR도 상위 GetData에서 처리. 여기서는 “이상치”를 한 번 넣어둘 수 있음.
      status.right_online_status = 2;
      status.left_online_status = 2;
      break;
  }
}

}  // namespace loadcell_sensor