cmake_minimum_required(VERSION 3.16)
project(loadcell_controller LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
endif()

# --- ROS2 ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_driver_interface REQUIRED)
find_package(builtin_interfaces REQUIRED)

# --- Dependencies: libzmq (C library) ---
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)

# --- cppzmq (header-only): zmq.hpp must exist ---
find_path(CPPZMQ_INCLUDE_DIR
  NAMES zmq.hpp
)
if(NOT CPPZMQ_INCLUDE_DIR)
  message(FATAL_ERROR "zmq.hpp not found. Install cppzmq headers (e.g., cppzmq / libcppzmq-dev).")
endif()

# --- msgpack (header-only): msgpack.hpp must exist ---
find_path(MSGPACK_INCLUDE_DIR
  NAMES msgpack.hpp
)
if(NOT MSGPACK_INCLUDE_DIR)
  message(FATAL_ERROR "msgpack.hpp not found. Install msgpack headers (e.g., libmsgpack-dev / msgpack-cxx).")
endif()

# ---- Shared prefix paths (RFID_Driver/install) ----
# ros2_controller/ 와 install/ 은 같은 레벨
set(INSTALL_SHARED_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../install")
set(ZEROMQ_PUB_DIR "${INSTALL_SHARED_PREFIX_DIR}/include/zeromq/publisher")

# ============================================================
# Target: ${PROJECT_NAME}_node
# ============================================================
add_executable(${PROJECT_NAME}_node
  "${CMAKE_CURRENT_LIST_DIR}/src/data_builder.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/src/loadcell_controller_node.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/src/main_controller.cpp"

  # shared ZMQ publisher bytes implementation
  "${ZEROMQ_PUB_DIR}/zmq_publisher_bytes.cpp"
)

target_include_directories(${PROJECT_NAME}_node PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/include"

  # shared ZMQ publisher headers (zmq_publisher_msgpack.hpp, zmq_publisher_bytes.h)
  "${ZEROMQ_PUB_DIR}"

  # header-only deps
  "${MSGPACK_INCLUDE_DIR}"
  "${CPPZMQ_INCLUDE_DIR}"

  # libzmq include dirs from pkg-config
  ${ZMQ_INCLUDE_DIRS}
)

target_compile_options(${PROJECT_NAME}_node PRIVATE
  ${ZMQ_CFLAGS_OTHER}
)

target_link_directories(${PROJECT_NAME}_node PRIVATE
  ${ZMQ_LIBRARY_DIRS}
)

# ament_target_dependencies가 내부에서 plain target_link_libraries를 사용하므로
# 여기서도 plain으로 통일(키워드 PRIVATE/PUBLIC 사용 금지)
target_link_libraries(${PROJECT_NAME}_node
  ${ZMQ_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  sensor_driver_interface
  builtin_interfaces
)

# ---- install ----
install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
