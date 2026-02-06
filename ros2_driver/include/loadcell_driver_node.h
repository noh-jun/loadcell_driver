#pragma once

#include "data_result.h"
#include "sensor_driver_node.h"
#include <memory>

class LoadCell485;

namespace sensor_driver_base {
class LoadCellDriverNode : public SensorDriverNode {
public:
  LoadCellDriverNode();
  ~LoadCellDriverNode();

  void ParamChange() override;
  bool IsConnected() override;
  void Connect() override;
  void GetData(DataResult&);
  void PrintPublishData(MetaMsg& msg);

  private:
  std::unique_ptr<LoadCell485> loadcell_;
  std::string loadcell_port_;
  int loadcell_address_;
};
} // namespace sensor_driver_base