#!/bin/bash
######################################################################################
# Build and Flash
#
# SYNOPSIS
#   ./build.sh [cm7|cm4] [-startup|-build|-b|-flash|-f]
#
# DESCRIPTION
#   This script will build and flash the CM4 and CM7 microprocessors. When using the 
#   script you will traditionally use it in one of 3 configurations: startup, build, or
#   flash. Startup is agnostic of the processor selection but for build and flash, you
#   will need to specify the processor.
#
# PROCESSORS
#   cm7              : The CM7 microprocessor.
#   cm4              : The CM4 microprocessor.
#
# OPTIONS
#   -s, --startup    : Install dependencies and configure both processors.
#   -b, --build      : Build the selected processor(s).
#   -f, --flash      : Flash the selected processor(s).
#   
# EXAMPLE
#   ./build.sh -s
#       Install dependencies and configure the project.
#   ./build.sh cm4 -b
#       Build the CM4 processor firmware.
#   ./build.sh cm7 -f
#       Flash the CM7 processor firmware.
#   ./build.sh cm4 cm7 -b -f
#       Build and flash both the CM4 and CM7 processor firmware.
######################################################################################
set -e

TOOLCHAIN="$(pwd)/gcc-arm-none-eabi.cmake"

cm4=0
cm7=0
startup=0
build=0
flash=0

while [[ $# -gt 0 ]]; do
    case "$1" in
        cm7) cm7=1 ;;
        cm4) cm4=1 ;;
        --startup|-s) startup=1 ;;
        --build|-b) build=1 ;;
        --flash|-f) flash=1 ;;
        *)
            if [[ $1 != "--help" && $1 != "-h" ]]; then printf "Unknown option: $1\n"; fi
            printf "\n"
            awk '/^#{80,}/{flag=!flag; next} flag' $0 | sed 's/#//g'
            printf "\n"
            exit 0
            ;;
    esac
    shift
done

if [[ $startup -eq 1 ]]; then
    printf "\n===========================================================================\n"
    printf "Installing dependencies...\n"
    sudo apt-get install -y gcc-arm-none-eabi ninja-build cmake openocd
fi

if [[ $cm4 -eq 1 || $startup -eq 1 ]]; then
    printf "\n===========================================================================\n"
    printf "Configuring CM4...\n"
    cmake --no-warn-unused-cli -B CM4/build \
        -DCMAKE_BUILD_TYPE=Debug \
        -DCMAKE_TOOLCHAIN_FILE="$TOOLCHAIN" \
        -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
        CM4

    if [[ $build -eq 1 ]]; then
        printf "\n===========================================================================\n"
        printf "Building CM4...\n"
        cmake --build CM4/build
    fi

    if [[ $flash -eq 1 ]]; then
        printf "\n===========================================================================\n"
        printf "Flashing CM4...\n"
        openocd -f interface/stlink.cfg \
                -f target/stm32h7x.cfg \
                -c "program CM4/build/Autonomaus_Sailboat_CM4.elf verify reset exit"
    fi
fi

if [[ $cm7 -eq 1 || $startup -eq 1 ]]; then
    printf "\n===========================================================================\n"
    printf "Configuring CM7...\n"
    cmake --no-warn-unused-cli -B CM7/build \
        -DCMAKE_BUILD_TYPE=Debug \
        -DCMAKE_TOOLCHAIN_FILE="$TOOLCHAIN" \
        -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
        CM7

    if [[ $build -eq 1 ]]; then
        printf "\n===========================================================================\n"
        printf "Building CM7...\n"
        cmake --build CM7/build
    fi

    if [[ $flash -eq 1 ]]; then
        printf "\n===========================================================================\n"
        printf "Flashing CM7...\n"
        openocd -f interface/stlink.cfg \
                -f target/stm32h7x.cfg \
                -c "program CM7/build/Autonomaus_Sailboat_CM7.elf verify reset exit"
    fi
fi
