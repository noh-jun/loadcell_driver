#include <atomic>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <thread>

#include <zmq.hpp>

#include "install/declare/data_type.h"
#include "zmq_subscriber_msgpack.hpp"

namespace {
constexpr const char* kEndpoint = "tcp://localhost:5556";
constexpr const char* kTopic = "rfid";
}  // namespace

/**
 * @brief ZMQ SUB로 RFID::DataResult를 구독하여 수신 내용을 출력하는 샘플 엔트리포인트.
 *
 * MsgPackSubscriber<RFID::DataResult>를 생성해 endpoint/topic을 구독하고, 수신된 메시지를 표준 출력으로 출력한다.
 * 검증은 장시간 실행하며 종료는 Ctrl+C(SIGINT)로 수행한다.
 *
 * @return 정상 종료 시 0이고 예외 발생 시 1이다.
 */
int main() {
  try {
    zmq::context_t context(1);

    std::atomic<std::uint64_t> received_count{0};

    zmq_pub::MsgPackSubscriber<RFID::DataResult> subscriber(
        context,
        std::string(kEndpoint),
        std::string(kTopic),
        /*max_queue_size=*/100,
        [&received_count](RFID::DataResult&& msg) {
          const std::uint64_t n = ++received_count;

          std::cout << "[SUB] recv #" << n << " state=" << static_cast<int>(msg.driver_state)
                    << " ret_code=" << msg.ret_code << " tags=" << msg.tags.size();

          if (!msg.tags.empty()) {
            std::cout << " epc=" << msg.tags[0].epc << " rssi=" << msg.tags[0].rssi
                      << " readCount=" << msg.tags[0].readCount << " antenna=" << msg.tags[0].antenna
                      << " ts=" << msg.tags[0].timeStamp;
          }

          std::cout << "\n";
        },
        [](const std::string& error_message) {
          std::cerr << "[SUB][ERR] " << error_message << "\n";
          return true;
        });

    std::cout << "[SUB] connect=" << kEndpoint << " topic=" << kTopic << "\n";
    std::cout << "[SUB] Press Ctrl+C to exit.\n";

    while (true) {
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    subscriber.Stop();

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    std::cout << "[SUB] done, received=" << received_count.load() << "\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "[SUB][ERR][EXCEPTION] " << e.what() << "\n";
    return 1;
  }
}
