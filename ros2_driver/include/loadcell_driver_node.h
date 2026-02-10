#pragma once

#include "data_result.h"
#include "sensor_driver_node.h"
#include <memory>
#include <sensor_driver_interface/msg/weight_meta.hpp>

namespace loadcell_comm {
class LoadCell485;
}

namespace loadcell_sensor {
class LoadCellDriverNode : public sensor_driver_base::SensorDriverNode {
public:
  LoadCellDriverNode();
  ~LoadCellDriverNode();

  void ParamChange() override;
  bool IsConnected() override;
  void Connect() override;
  void GetData(sensor_driver_base::DataResult &) override;
  void PrintPublishData(MetaMsg &msg) override;

private:
  std::unique_ptr<loadcell_comm::LoadCell485> loadcell_;
  std::string loadcell_port_;
  int loadcell_address_;
  int loadcell_baudrate_;
  int loadcell_databits_;
  std::string loadcell_parity_;
  int loadcell_stopbits_;
};
} // namespace loadcell_sensor