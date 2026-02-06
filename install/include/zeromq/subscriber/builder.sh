#!/usr/bin/env bash
set -euo pipefail # 에러 발생 시 즉시 종료

# ================================
# usage 출력 함수
# ================================
usage() {
  echo "Usage: $0 [-b <build path>] [-s <source path>] [-t <Debug|Release|RelWithDebInfo>]"
  echo ""
  echo "Options:"
  echo "  -b   Build directory path (default: Build/)"
  echo "  -s   Source directory path *default: ./)"
  echo "  -t   Build type (optional, default: Debug)"
  exit 1
}

# ================================
# 기본 값
# ================================
BUILD_PATH="Build"
SRC_PATH="."
BUILD_TYPE="Debug"

# ================================
# 옵션 파싱
# ================================
while getopts "b:s:t:h:" opt; do
    case $opt in
        b) BUILD_PATH=$OPTARG ;;
        s) SRC_PATH=$OPTARG ;;
        t) BUILD_TYPE=$OPTARG ;;
        h) usage ;;
        *) usage ;;
    esac
done

# ================================
# 빌드 타입 정규화/검증
# ================================
case "$BUILD_TYPE" in
  debug|Debug) BUILD_TYPE="Debug" ;;
  release|Release) BUILD_TYPE="Release" ;;
  RelWithDebInfo|relwithdebinfo|Relwithdebinfo) BUILD_TYPE="RelWithDebInfo" ;;
  *)
    echo "Error: Invalid build type '$BUILD_TYPE'"
    echo "Allowed values: Debug, Release, RelWithDebInfo"
    exit 1
    ;;
esac

echo "===================================="
echo " Build path : $BUILD_PATH"
echo " Source path : $SRC_PATH"
echo " Build type : $BUILD_TYPE"
echo "===================================="

# ================================
# Clean
# ================================
echo "[1/3] Removing build directory: $BUILD_PATH"
rm -rf "$BUILD_PATH"
mkdir -p "$BUILD_PATH"

# ================================
# Configure
# ================================
echo "[2/3] Configuring (CMAKE_BUILD_TYPE=$BUILD_TYPE)"
cmake -S "$SRC_PATH" -B "$BUILD_PATH" -DCMAKE_BUILD_TYPE="$BUILD_TYPE"

# ================================
# Build
# ================================
echo "[3/3] Building"
cmake --build "$BUILD_PATH"

echo "Done."
