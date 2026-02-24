#include <exception>
#include <memory>

#include "loadcell_driver_node.h"
#include "loadcell_simulator_node.h"
#include "rclcpp/rclcpp.hpp"

/**
 * @brief 프로그램 진입점 함수.
 *
 * ROS2 런타임을 초기화한 뒤 SensorDriverBaseNode를 생성하여 스핀한다.
 * 노드가 종료되면 ROS2 런타임을 정리하고 정상 종료 코드를 반환한다.
 *
 * @param argc 프로그램 인자 개수.
 * @param argv 프로그램 인자 배열.
 * @return 정상 종료 시 0을 반환한다.
 */
int main(int argc, char* argv[]) {
  try {
    /** ROS2 런타임을 초기화한다. */
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor executor;

    /** SensorDriverBaseNode를 생성 */
    auto node = std::make_shared<loadcell_sensor::LoadCellSimulatorNode>();
    // auto node = std::make_shared<loadcell_sensor::LoadCellDriverNode>();

    /** 초기화 */
    node->Initialize();

    /** node 추가 */
    executor.add_node(node);

    /** SensorDriverBaseNode를 생성하여 콜백 처리를 시작한다. */
    executor.spin();

    /** ROS2 런타임을 종료하고 자원을 정리한다. */
    rclcpp::shutdown();

    /** 정상 종료를 의미하는 종료 코드를 반환한다. */
    return 0;
  }
  catch (const std::exception& e) {
    std::cerr << "(main-print) exception: " << e.what() << std::endl;

    /** ROS2 런타임을 종료하고 자원을 정리한다. */
    rclcpp::shutdown();

    return 1;
  }
}