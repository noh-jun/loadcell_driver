#ifndef DATA_BUILDER_H
#define DATA_BUILDER_H

#include <sensor_driver_interface/msg/weight_meta.hpp>
#include "zmq_data_type.h"
#include "zmq_publisher_msgpack.hpp"

using MetaMsg = sensor_driver_interface::msg::WeightMeta;

namespace loadcell_controller {
class DataBuilder {
public:
    DataBuilder();
    ~DataBuilder();

    LOADCELL::DataResult Build(const MetaMsg& message);

private:
    void PrintDebug(const MetaMsg& message) const noexcept;
    void PrintDebug(const LOADCELL::DataResult& data_result) const noexcept;
    std::uint64_t time_to_uint64(const builtin_interfaces::msg::Time& t) const noexcept;
    std::string time_to_string(const builtin_interfaces::msg::Time& t) const noexcept;
    std::string time_to_string(std::uint64_t t) const noexcept;
    std::string state_to_string(std::uint8_t state) const noexcept;
};

} // namespace loadcell_controller

#endif // DATA_BUILDER_H