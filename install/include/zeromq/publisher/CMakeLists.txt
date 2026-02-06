cmake_minimum_required(VERSION 3.16)
project(zmq_publisher_test LANGUAGES CXX)

# 상위 Package Dir 경로 설정 (rfid_driver package 기준)
set(PACKAGE_DIR ${CMAKE_CURRENT_LIST_DIR}/../../../../)

# PkgConfig 포함
find_package(PkgConfig REQUIRED)

# ZMQ Publisher Test 코드 생성
add_executable(zmq_publisher_test
  main.cpp
  zmq_publisher_bytes.cpp
)

# zmq_publisher_test 의 개발 언어 및 표준 선택
target_compile_features(zmq_publisher_test PRIVATE cxx_std_17)

# zmq_publisher_test 의 헤더 파일을 찾을 디렉터리를 지정
target_include_directories(zmq_publisher_test PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  ${PACKAGE_DIR}
)

# ZeroMQ (libzmq)
pkg_check_modules(PC_ZMQ REQUIRED libzmq)
target_include_directories(zmq_publisher_test PRIVATE ${PC_ZMQ_INCLUDE_DIRS})
target_link_directories(zmq_publisher_test PRIVATE ${PC_ZMQ_LIBRARY_DIRS})
target_link_libraries(zmq_publisher_test PRIVATE ${PC_ZMQ_LIBRARIES})
