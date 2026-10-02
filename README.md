# AUTONOMOUS SAILBOAT



## REPOSITORY

This repository contains the code and scripts for the autonomous sailboat project, including firmware for the CM4 and CM7 processors, as well as startup routines and hardware initialization for the servos and sensors. The main directories and files are enumerated below.

### CM4

This contains all of the code specific to the CM4 core. Below I've listed the important directories and files.

* `App` directory - Contains all of the developer code for the CM4 core.
* `build` directory - Contains built artifacts, ignored by version control.
* `Core` directory - Contains the STM32 Cube files used to start the core and the developer code.
* `Core/Inc/stm32h7xx_hal_conf.h` - This controls what HAL drivers are enabled and what hal drivers are not.
* `Core/Src/main.c` - I assume you know why main is important.
* `CMakeLists.txt` - The CMake build configuration file for the CM4 core, this is been designed to dynamically build any new files in the `App` and `Drivers` directories. It will link any files that are in the `Core`, `App`, `Drivers`, and `Common` directories.

I have not tested the lora fully but it should work with the current setup.

### CM7

This contains all of the code specific to the CM7 core. Its design and layout will mirror that of the CM4 core with one exception. The `CMakeLists.txt` file will also compile and include any files in the `Middlewares` directory.

### Common

This contains all of the code that is shared between the CM4 and CM7 cores.

`Inc/shared_configuration.h` - This file is the top header file of the repo and is shared accross everything. All it contains are a bunch of defines that you can use to control what compiles and what runs. It has three sections.

1) Shared Configurations
2) CM4 Configuration
3) CM7 Configuration

### docs

Kinda a hold over from 3992 but I figured it would be good to have a dedicated place where we store docs so I kept it here.

### Drivers

HAL Drivers for the STM32 microcontrollers. I've included all of the drivers I could find so if you program isn't compiling it is probably because you need to enable the corresponding driver in `Core/Inc/stm32h7xx_hal_conf.h` file of your respective core.

### hardware

This directory contains a KiCAD project for the hardware design of the autonomous sailboat.

### Middlewares

Middlewares for the STM32, right now all we have is RTOS and that is probably all we are ever going to have.

### build.sh

This is the new build script for the repository. It can be used to build and flash both cores independently or together. Use the following command to get details on how to use it. It is unix so windows any gonna run it.

```bash
./build.sh --help
```

### common-sources.cmake

Both of the `CMakeLists.txt` files reference this file. It is a central source of truth for any CMake commands that need to be shared by both cores.

### PROJECT.md

Contains the project plan and rough information about how to implement it.



## DEBUG

This STM32 is equiped with a virtual serial port that lets you connect to its serial debug port directly over the USB you use to flash, no extra nonsense needed. To run it you need to use the following commands.

linux
```bash
screen $(ls /dev/ttyACM* | head -1) 115200 # To start the screen
# To stop the screen click "ctrl+A" then "k" then "y".
```