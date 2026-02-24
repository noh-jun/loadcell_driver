#ifndef DATA_TYPE_H
#define DATA_TYPE_H

#include <cstdint>
#include <string>
#include <vector>

#include <msgpack.hpp>

namespace LOADCELL {

enum DRIVER_STATE : uint8_t { OK = 0, EMPTY = 1, ERROR = 2 };

struct LoadcellStatus {
  double gross_weight = 0.0;
  double right_weight = 0.0;
  double left_weight = 0.0;

  uint8_t right_battery_percent = 0;
  uint8_t right_charge_status = 0; // 0: uncharged, 1: charging
  uint8_t right_online_status = 0; // 0: online, 1: offline, 2: hw failure

  uint8_t left_battery_percent = 0;
  uint8_t left_charge_status = 0; // 0: uncharged, 1: charging
  uint8_t left_online_status = 0; // 0: online, 1: offline, 2: hw failure

  uint8_t gross_net_mark = 0;        // 0: gross, 1: net
  uint8_t overload_mark = 0;         // 0: not overloaded, 1: overload
  uint8_t out_of_tolerance_mark = 0; // 0: ok, 1: left, 2: right

  MSGPACK_DEFINE(gross_weight, right_weight, left_weight,
                right_battery_percent, right_charge_status, right_online_status,
                left_battery_percent, left_charge_status, left_online_status,
                gross_net_mark, overload_mark, out_of_tolerance_mark)
};

struct DataResult {
  std::string driver_instance_id;
  std::uint64_t seq_no;
  std::uint64_t pub_timestamp;
  std::uint8_t driver_state;
  std::int32_t ret_code;
  std::string driver_err_msg;
  LoadcellStatus status;

  MSGPACK_DEFINE(driver_instance_id, seq_no, pub_timestamp, driver_state, ret_code, driver_err_msg, status)
};

} // namespace LOADCELL

#endif // DATA_TYPE_H