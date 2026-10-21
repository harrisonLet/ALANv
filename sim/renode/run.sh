#!/bin/bash
# Build the CM7 firmware for the virtual board (with simulated sensors) and start Renode.
#
#   sim/renode/run.sh               build, then open Renode with its windows
#   sim/renode/run.sh --headless 10 build, then run 10 virtual seconds and print the console
#   sim/renode/run.sh --no-build    skip the build
#   sim/renode/run.sh --sensors imu,encoder,gps   choose sensors (default: wind,imu,encoder,gps)
#   sim/renode/run.sh --build-only  build and stop (used by the dashboard)
#
# The sim build goes to sim/renode/build/cm7 and turns the sensors on with -D flags,
# so Common/Inc/shared_configuration.h is not modified.
set -e
cd "$(dirname "$0")/../.."

RENODE="${RENODE:-$(command -v renode || echo /Applications/Renode.app/Contents/MacOS/renode)}"
BUILD_DIR=sim/renode/build/cm7
sensors="wind,imu,encoder,gps"

build=1
build_only=0
headless=0
seconds=10
while [[ $# -gt 0 ]]; do
    case "$1" in
        --no-build) build=0 ;;
        --build-only) build_only=1 ;;
        --headless) headless=1; if [[ "$2" =~ ^[0-9]+$ ]]; then seconds="$2"; shift; fi ;;
        --sensors)
            # Accept "wind,encoder,gps" as well as "wind, encoder, gps"
            sensors="$2"; shift
            while [[ "$sensors" == *, && $# -gt 1 ]]; do sensors+="$2"; shift; done
            sensors="${sensors// /}"
            ;;
        -h|--help) sed -n '2,11p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "Unknown option: $1"; echo; sed -n '2,11p' "$0" | sed 's/^# \{0,1\}//'; exit 1 ;;
    esac
    shift
done

SIM_FLAGS="-DRENODE_SIM -DCM7_CONSOLE_ENABLE"
[[ ",$sensors," == *",wind,"* ]]    && SIM_FLAGS+=" -DCM7_SENSOR_WIND_ENABLE"
[[ ",$sensors," == *",imu,"* ]]     && SIM_FLAGS+=" -DCM7_SENSOR_MAGNETOMETER_ENABLE"
[[ ",$sensors," == *",encoder,"* ]] && SIM_FLAGS+=" -DCM7_SENSOR_ENCODER_ENABLE"
[[ ",$sensors," == *",gps,"* ]]     && SIM_FLAGS+=" -DCM7_SENSOR_GPS_ENABLE"

if [[ $build -eq 1 ]]; then
    echo "Building CM7 for Renode with: $SIM_FLAGS"
    cmake -Wno-unused-cli -S CM7 -B "$BUILD_DIR" \
        -DCMAKE_BUILD_TYPE=Debug \
        -DCMAKE_TOOLCHAIN_FILE="$(pwd)/gcc-arm-none-eabi.cmake" \
        -DCMAKE_C_FLAGS="$SIM_FLAGS" > /dev/null 2>&1
    if ! cmake --build "$BUILD_DIR" -- -j8 > "$BUILD_DIR/build.log" 2>&1; then
        grep -E "error" "$BUILD_DIR/build.log" | head -20
        echo "Build FAILED (full log: $BUILD_DIR/build.log)"
        exit 1
    fi
    grep -E "FLASH:|RAM:" "$BUILD_DIR/build.log" | grep -v ITCM || true
fi
[[ $build_only -eq 1 ]] && exit 0

if [[ $headless -eq 1 ]]; then
    "$RENODE" --disable-gui --console -e "include @sim/renode/alan_cm7.resc; emulation RunFor \"00:00:$(printf '%02d' "$seconds")\"; quit" 2>&1 \
        | sed 's/\x1b\[[0-9;]*m//g' | grep -E "usart3:|ERROR|error" | sed -E 's/.*usart3: \[[^]]*\] //'
else
    "$RENODE" sim/renode/alan_cm7.resc
fi
