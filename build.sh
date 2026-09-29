#!/bin/bash
set -e

TOOLCHAIN="$(pwd)/gcc-arm-none-eabi.cmake"

build_cm4=0
build_cm7=0
startup=0

while [[ $# -gt 0 ]]; do
    case "$1" in
        -cm7) build_cm7=1 ;;
        -cm4) build_cm4=1 ;;
        -startup) startup=1 ;;
        *)
            echo "Usage: $0 [-cm7|-cm4|-startup]"
            exit 1
            ;;
    esac
    shift
done

if [[ $startup -eq 1 ]]; then
    printf "\n===========================================================================\n"
    printf "Installing dependencies...\n"
    sudo apt-get install -y gcc-arm-none-eabi ninja-build cmake openocd

    printf "\n===========================================================================\n"
    printf "Configuring CM7...\n"
    cmake --no-warn-unused-cli -B CM7/build \
        -DCMAKE_BUILD_TYPE=Debug \
        -DCMAKE_TOOLCHAIN_FILE="$TOOLCHAIN" \
        -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
        CM7

    printf "\n===========================================================================\n"
    printf "Configuring CM4...\n"
    cmake --no-warn-unused-cli -B CM4/build \
        -DCMAKE_BUILD_TYPE=Debug \
        -DCMAKE_TOOLCHAIN_FILE="$TOOLCHAIN" \
        -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
        CM4
fi

if [[ $build_cm4 -eq 1 || $startup -eq 1 ]]; then
    printf "\n===========================================================================\n"
    printf "Building CM4...\n"
    cmake --build CM4/build
fi

if [[ $build_cm7 -eq 1 || $startup -eq 1 ]]; then
    printf "\n===========================================================================\n"
    printf "Building CM7...\n"
    cmake --build CM7/build
fi
