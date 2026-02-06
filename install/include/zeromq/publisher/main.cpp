#include <chrono>
#include <cstdint>
#include <iostream>
#include <thread>

#include <zmq.hpp>

#include "install/declare/data_type.h"
#include "zmq_publisher_msgpack.hpp"

namespace {
/** @brief Publisher가 bind할 endpoint. */
constexpr const char* kEndpoint = "tcp://*:5556";
/** @brief Publisher가 사용할 topic 문자열. */
constexpr const char* kTopic = "rfid_msgs";
}  // namespace

/**
 * @brief 시퀀스 번호를 포함한 샘플 RFID::DataResult 메시지를 생성한다.
 * driver_state는 OK로 설정하고, ret_code는 0으로 설정한다.
 * tagInfo 1개를 생성하여 out.tags에 추가하며, epc/readCount/timeStamp에 sequence_no를 반영한다.
 * @param sequence_no 샘플 값 생성에 사용할 시퀀스 번호.
 * @return 구성된 RFID::DataResult.
 * @note 본 함수는 퍼블리시 동작 검증을 위한 테스트 데이터 생성 목적이다.
 */
static RFID::DataResult MakeSample(std::uint64_t sequence_no) {
  RFID::DataResult out{};
  out.driver_state = static_cast<std::uint8_t>(RFID::DRIVER_STATE::OK);
  out.ret_code = 0;
  out.driver_err_msg = "";

  RFID::tagInfo tag{};
  tag.epc = "E2000017221101441890" + std::to_string(sequence_no);
  tag.rssi = -42;
  tag.readCount = static_cast<std::uint32_t>(sequence_no);
  tag.antenna = 1;
  tag.timeStamp = static_cast<std::uint64_t>(sequence_no * 1000ULL);

  out.tags.push_back(std::move(tag));
  return out;
}

/**
 * @brief MsgPackPublisher를 이용해 샘플 RFID 메시지를 주기적으로 발행하는 테스트 프로그램.
 * ZMQ context를 생성하고, MsgPackPublisher<RFID::DataResult>를 endpoint/topic으로 초기화한다.
 * SUB가 connect/subscribe를 완료할 시간을 주기 위해 초기 워밍업 딜레이를 둔다.
 * 일정 횟수(kPublishCount) 동안 샘플 메시지를 생성하여 Publish로 전송하고, 전송한 내용을 stdout에 출력한다.
 * 예외 발생 시 에러를 출력하고 비정상 종료 코드(1)를 반환한다.
 * @return 정상 종료 시 0, 예외 발생 시 1.
 * @note PUB/SUB 패턴에서 SUB는 구독 설정 이전에 전송된 메시지를 놓칠 수 있으므로, 워밍업 딜레이는 첫 메시지 유실을 줄이기 위한 목적이다.
 */
int main() {
  try {
    zmq::context_t context(1);

    zmq_pub::MsgPackPublisher<RFID::DataResult> publisher(
        context, std::string(kEndpoint), std::string(kTopic));

    std::cout << "[PUB] bind=" << publisher.Endpoint()
              << " topic=" << publisher.Topic() << "\n";

    // SUB가 connect/subscribe를 완료할 시간을 조금 준다(첫 publish 유실 방지용 워밍업).
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    constexpr std::uint64_t kPublishCount = 20;
    for (std::uint64_t i = 0; i < kPublishCount; ++i) {
      const RFID::DataResult msg = MakeSample(i);

      publisher.Publish(msg);

      std::cout << "[PUB] sent #" << i
                << " state=" << static_cast<int>(msg.driver_state)
                << " ret_code=" << msg.ret_code
                << " tags=" << msg.tags.size()
                << " epc=" << (msg.tags.empty() ? "" : msg.tags[0].epc)
                << "\n";

      std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    std::cout << "[PUB] done\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "[PUB][ERR] " << e.what() << "\n";
    return 1;
  }
}
