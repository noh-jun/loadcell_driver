#pragma once

#include <cstdint>
#include <string>

#include <loadcell_comm/loadcell_485.h>

namespace sensor_driver_base {

/**
 * @brief Sensor 드라이버가 퍼블리시하는 데이터 결과를 나타내는 구조체.
 * 공용 값 : 
 * driver_state : 드라이버 상태 값 (예: OK, EMPTY, ERROR 등).
 * uint8 DRIVER_STATE_OK    = 0
 * uint8 DRIVER_STATE_EMPTY = 1
 * uint8 DRIVER_STATE_ERROR = 2
 * return_code : 드라이버 동작 결과를 나타내는 정수 코드 (0: 정상, 음수: 경고, 양수: 오류).
 * driver_err_msg : 드라이버 오류 발생 시 설명 메시지.
 * 개별 값 :
 * status : LoadCell 상태 정보를 포함하는 LoadCellStatus 구조체.
 */
enum DRIVER_STATE : uint8_t { OK = 0, EMPTY = 1, ERROR = 2 };

struct DataResult {
  std::uint8_t driver_state = DRIVER_STATE::ERROR;
  std::int32_t return_code = 0;
  std::string driver_err_msg = "";
  loadcell_comm::LoadCellStatus status;
};
}  // namespace sensor_driver_base