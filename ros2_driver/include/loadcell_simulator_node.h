#pragma once

#include "data_result.h"
#include "sensor_driver_node.h"

#include <cstdint>
#include <memory>
#include <random>
#include <string>

#include <sensor_driver_interface/msg/weight_meta.hpp>

namespace loadcell_sensor {

class LoadCellSimulatorNode : public sensor_driver_base::SensorDriverNode {
public:
  LoadCellSimulatorNode();
  ~LoadCellSimulatorNode();

  void ParamChange() override;
  bool IsConnected() override;
  void Connect() override;
  void GetData(sensor_driver_base::DataResult& out) override;
  void PrintPublishData(MetaMsg& msg) override;

private:
  enum class Scenario : std::uint8_t {
    kNormal = 0,
    kEmpty = 1,
    kError = 2,
    kRightOffline = 3,
    kLeftOffline = 4,
    kRightHwFailure = 5,
    kLeftHwFailure = 6,
    kOverload = 7,
    kOutOfToleranceLeft = 8,
    kOutOfToleranceRight = 9
  };

  Scenario ParseScenario(const std::string& scenario_text) const;

  double RandomDouble(double min_value, double max_value);
  std::uint8_t RandomU8(std::uint8_t min_value, std::uint8_t max_value);

  void FillStatusByScenario(Scenario scenario, loadcell_comm::LoadCellStatus& status);

private:
  bool connected_ = false;

  // parameters
  std::string scenario_text_;
  double weight_min_kg_ = 0.0;
  double weight_max_kg_ = 200.0;
  double weight_noise_kg_ = 1.0;

  std::uint8_t battery_min_percent_ = 20;
  std::uint8_t battery_max_percent_ = 100;

  // RNG
  std::mt19937 rng_;
};

}  // namespace loadcell_sensor