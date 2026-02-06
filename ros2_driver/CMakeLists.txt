cmake_minimum_required(VERSION 3.8)
project(loadcell_driver LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_driver_interface REQUIRED)

# ------------------------------------------------------------------------------
# External install bundle (weight_comm/install/weight_comm)
#   - Headers : <repo_root>/install/weight_comm/include/...
#   - Libs    : <repo_root>/install/weight_comm/lib/libloadcell_485_comm.so
#
# CMakeLists.txt is in: weight_driver/ros2_driver/CMakeLists.txt
# repo root is one level up from this file.
# ------------------------------------------------------------------------------
get_filename_component(WEIGHT_DRIVER_REPO_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
set(LOADCELL_BUNDLE_ROOT "${WEIGHT_DRIVER_REPO_ROOT}/install/weight_comm")
set(LOADCELL_BUNDLE_INCLUDE_DIR "${LOADCELL_BUNDLE_ROOT}/include")
set(LOADCELL_BUNDLE_LIB_DIR "${LOADCELL_BUNDLE_ROOT}/lib")

find_library(LOADCELL_485_COMM_LIB
  NAMES loadcell_485_comm
  PATHS "${LOADCELL_BUNDLE_LIB_DIR}"
  NO_DEFAULT_PATH
)

if(NOT LOADCELL_485_COMM_LIB)
  message(FATAL_ERROR
    "Failed to find libloadcell_485_comm in: ${LOADCELL_BUNDLE_LIB_DIR}\n"
    "Expected something like: ${LOADCELL_BUNDLE_LIB_DIR}/libloadcell_485_comm.so"
  )
endif()

set(LOADCELL_DRIVER_SRC
  src/data_builder.cpp
  src/sensor_driver_node.cpp
  src/loadcell_driver_node.cpp
  src/unique_key_generator.cpp
)

# 실행 파일 생성
add_executable(${PROJECT_NAME}_node
  src/main.cpp
  ${LOADCELL_DRIVER_SRC}
)

target_include_directories(${PROJECT_NAME}_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${LOADCELL_BUNDLE_INCLUDE_DIR}
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  sensor_driver_interface
)

target_link_libraries(${PROJECT_NAME}_node
  ${LOADCELL_485_COMM_LIB}
)

# 런타임 로더 설정
# - build tree에서 실행: bundle lib 경로를 rpath에 추가
# - install tree에서 실행: 같은 디렉터리($ORIGIN)에서 lib 찾도록 설정
set_target_properties(${PROJECT_NAME}_node PROPERTIES
  BUILD_RPATH "${LOADCELL_BUNDLE_LIB_DIR}"
  INSTALL_RPATH "$ORIGIN"
)

# 실행 파일 설치
install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# (권장) 런타임 안정성을 위해 외부 bundle의 .so를 패키지와 함께 설치
# - 실행 파일과 같은 디렉터리에 두면 INSTALL_RPATH="$ORIGIN"로 바로 로딩 가능
# 버전 파일명은 실제 빌드 산출물에 맞게 조정하십시오.
if(EXISTS "${LOADCELL_BUNDLE_LIB_DIR}/libloadcell_485_comm.so.0.1.0")
  install(FILES
    "${LOADCELL_BUNDLE_LIB_DIR}/libloadcell_485_comm.so.0.1.0"
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

# 심볼릭 링크 이름을 코드/링커 기대에 맞게 같이 설치(있을 때만)
if(EXISTS "${LOADCELL_BUNDLE_LIB_DIR}/libloadcell_485_comm.so.0")
  install(FILES
    "${LOADCELL_BUNDLE_LIB_DIR}/libloadcell_485_comm.so.0"
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

if(EXISTS "${LOADCELL_BUNDLE_LIB_DIR}/libloadcell_485_comm.so")
  install(FILES
    "${LOADCELL_BUNDLE_LIB_DIR}/libloadcell_485_comm.so"
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

ament_package()
