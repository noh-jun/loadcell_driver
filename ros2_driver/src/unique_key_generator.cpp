#include "unique_key_generator.h"

namespace sensor_driver_uk {

/**
 * @brief ISO8601 형식의 UTC 타임스탬프 문자열을 생성한다.
 * std::chrono::system_clock::time_point 값을 UTC 기준의 문자열로 변환한다.
 * 반환되는 문자열은 YYYY-MM-DDTHH:MM:SS.mmmZ 형식이며 밀리초 단위의 정밀도를 포함한다.
 * @param time_point 변환할 시스템 시각.
 * @return ISO8601 형식의 UTC 타임스탬프 문자열.
 */
std::string Time_2_String(const ChronoClock::time_point& time_point) {
  /** 전달된 시각을 밀리초 단위로 정밀도를 유지하여 변환한다. */
  const auto time_point_ms = Chrono::time_point_cast<Chrono::milliseconds>(time_point);

  /** 밀리초 부분을 추출한다. */
  const auto milliseconds_part = static_cast<int>(time_point_ms.time_since_epoch().count() % 1000);

  /** system_clock 시간을 time_t 형식으로 변환한다. */
  std::time_t time_value = ChronoClock::to_time_t(time_point);

  /** UTC 시간을 추출한다. */
  std::tm utc_tm{};
  gmtime_r(&time_value, &utc_tm);

  /** 날짜와 시간을 ISO8601 형식으로 포맷팅하고 밀리초 및 Z 접미사를 추가한다. */
  std::ostringstream stream;
  stream << std::put_time(&utc_tm, "%Y-%m-%dT%H:%M:%S") << "." << std::setw(3) << std::setfill('0') << milliseconds_part
         << "Z";

  return stream.str();
}

/**
 * @brief UUID 형식과 유사한 문자열을 생성한다.
 * 랜덤 값을 기반으로 16진수 문자열을 생성하여 8-4-4-4-12 형태의 UUID 유사 문자열을 반환한다.
 * RFC4122를 엄격히 따르지는 않지만 예제 및 인스턴스 식별 용도로 충돌 가능성이 충분히 낮다.
 * @return UUID 형태와 유사한 문자열.
 */
std::string Gen_UUID_2_String() {
  /** 난수 생성을 위한 random_device 와 64비트 Mersenne Twister 엔진을 생성 */
  std::random_device random_device;
  std::mt19937_64 random_engine(random_device());

  /** 64비트 난수 분포 생성 */
  std::uniform_int_distribution<std::uint64_t> distribution(0, std::numeric_limits<std::uint64_t>::max());

  /**
   * @brief 지정된 개수의 16진수 문자를 생성하는 람다 함수.
   * 난수 값을 4비트 단위(nibble)로 잘라 0~15 범위의 값을 16진수 문자로 변환한다.
   * @param hex_count 생성할 16진수 문자 개수.
   * @return 생성된 16진수 문자열.
   */
  auto NextHex = [&](int hex_count) -> std::string {
    std::ostringstream hex_stream;
    for (int index = 0; index < hex_count; ++index) {
      const int nibble = static_cast<int>(distribution(random_engine) & 0xFULL);
      hex_stream << std::hex << std::nouppercase << nibble;
    }
    return hex_stream.str();
  };

  /** UUID 표준 형식과 유사한 8-4-4-4-12 구조로 문자열을 조합 */
  std::ostringstream uuid_stream;
  uuid_stream << NextHex(8) << "-" << NextHex(4) << "-" << NextHex(4) << "-" << NextHex(4) << "-" << NextHex(12);

  /**< UUID 형식과 유사한 문자열을 반환한다. */
  return uuid_stream.str();
}

/**
 * @brief 드라이버 인스턴스 ID를 생성한다.
 * 센서 ID와 타임스탬프, UUID를 조합하여 고유한 드라이버 인스턴스 ID를 생성한다.
 * @param sensor_id 센서 식별자.
 * @return 생성된 드라이버 인스턴스 ID.
 */
std::string GenerateInstanceID(const std::string& sensor_id) {
  /** 드라이버 시작 시점을 현재 시스템 시간으로 획득한다. */
  const auto start_time_point = ChronoClock::now();

  /** UTC 기준의 시작 타임스탬프를 문자열로 변환한다. */
  const std::string start_timestamp_utc = Time_2_String(start_time_point);

  /** UUID 형식과 유사한 문자열을 생성한다. */
  const std::string uuid_string = Gen_UUID_2_String();

  /** 센서 ID, 타임스탬프, UUID를 구분자로 연결하여 최종 인스턴스 ID를 구성한다. */
  std::ostringstream stream;
  stream << sensor_id << "/" << start_timestamp_utc << "/" << uuid_string;

  /** 드라이버 인스턴스 ID를 반환한다. */
  return stream.str();
}

}  // namespace sensor_driver_uk