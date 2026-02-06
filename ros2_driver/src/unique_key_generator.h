#pragma once

#include <chrono>
#include <cstdint>
#include <ctime>
#include <iomanip>
#include <limits>
#include <random>
#include <sstream>
#include <string>

namespace Chrono = std::chrono;
using ChronoClock = Chrono::system_clock;

namespace sensor_driver_uk {
/** 
* @brief Generates sequence numbers
* 드라이버가 퍼블리시를 시도할 때마다 증가하는 시퀀스 번호를 관리한다.
* 퍼블리시 결과가 OK, EMPTY, ERROR인지와 무관하게 모든 퍼블리시 시도에 대해 시퀀스 번호는 증가한다.
*/
class GenSeqNo {
 public:
  /** 
  * @brief Constructor 
  * 시퀀스 번호를 0으로 초기화한다.
  */
  GenSeqNo() : seq_no_(0) {};
  /**
   * @brief Destructor
   */
  ~GenSeqNo() = default;

  /**
    * @brief Returns the current sequence number
    * 내부에 저장된 현재 시퀀스 번호 값을 변경하지 않고 그대로 반환한다.
    * @return 현재 시퀀스 번호.
    */
  uint64_t Current() const { return seq_no_; }
  /**
   * @brief Increments the sequence number
   * 퍼블리시를 한 번 시도한 이후 반드시 호출되어 시퀀스 번호를 1 증가시킨다.
   * 퍼블리시 결과가 성공이든 실패든 상관없이 항상 증가한다.
   */
  void Increment() { ++seq_no_; }

 private:
  /** 
   * @brief Current sequence number
   * 현재 시퀀스 번호 값을 저장하는 변수.
   */
  uint64_t seq_no_;
};

/**
 * @brief ISO8601 형식의 UTC 타임스탬프 문자열을 생성한다.
 * std::chrono::system_clock::time_point 값을 UTC 기준의 문자열로 변환한다.
 * 반환되는 문자열은 YYYY-MM-DDTHH:MM:SS.mmmZ 형식이며 밀리초 단위의 정밀도를 포함한다.
 * @param time_point 변환할 시스템 시각.
 * @return ISO8601 형식의 UTC 타임스탬프 문자열.
 */
std::string Time_2_String(const ChronoClock::time_point& time_point);

/**
 * @brief UUID 형식과 유사한 문자열을 생성한다.
 * 랜덤 값을 기반으로 16진수 문자열을 생성하여 8-4-4-4-12 형태의 UUID 유사 문자열을 반환한다.
 * RFC4122를 엄격히 따르지는 않지만 예제 및 인스턴스 식별 용도로 충돌 가능성이 충분히 낮다.
 * @return UUID 형태와 유사한 문자열.
 */
std::string Gen_UUID_2_String();

/**
 * @brief 드라이버 인스턴스 ID를 생성한다.
 * 센서 ID와 타임스탬프, UUID를 조합하여 고유한 드라이버 인스턴스 ID를 생성한다.
 * @param sensor_id 센서 식별자.
 * @return 생성된 드라이버 인스턴스 ID.
 */
std::string GenerateInstanceID(const std::string& sensor_id);
}  // namespace sensor_driver_uk