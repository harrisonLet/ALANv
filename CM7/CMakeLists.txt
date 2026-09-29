cmake_minimum_required(VERSION 3.22)

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Compiler options
set(STM32_MCU_FLAGS  "-mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard " )

#Linker options
set(STM32_LINKER_SCRIPT stm32h755xx_flash_CM7.ld)
set(STM32_LINKER_OPTION  )

# Set the project name
set(CMAKE_PROJECT_NAME Autonomaus_Sailboat_CM7)
project(${CMAKE_PROJECT_NAME})

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Shared driver/BSP sources and include dirs (glob-based, replaces mx-generated.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/../common-sources.cmake)

# In order to use CMake for cross-compiling
set(CMAKE_CXX_COMPILER_FORCED "true")

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Core-specific macros
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    CORE_CM7
    USE_HAL_DRIVER
    STM32H755xx
    USE_PWR_DIRECT_SMPS_SUPPLY
    $<$<CONFIG:Debug>:DEBUG>
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${COMMON_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/App/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../Middlewares/Third_Party/FreeRTOS/Source/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2
    ${CMAKE_CURRENT_SOURCE_DIR}/../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
    ${CMAKE_CURRENT_SOURCE_DIR}/../Drivers/CMSIS/RTOS2/Include
)

# CubeMX-owned entry points - explicit since they never gain siblings
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Startup/startup_stm32h755xx_CM7.s
    Core/Src/main.c
    Core/Src/freertos.c
    Core/Src/stm32h7xx_it.c
    Core/Src/stm32h7xx_hal_msp.c
    Core/Src/sysmem.c
    Core/Src/syscalls.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../Common/Src/system_stm32h7xx_dualcore_boot_cm4_cm7.c
    ${COMMON_DRIVER_SOURCES}
)

# Auto-collect user sources - app code lives outside Core/, which is CubeMX-owned
file(GLOB USER_SOURCES CONFIGURE_DEPENDS "App/Src/*.c")
target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${USER_SOURCES})

# FreeRTOS sources - vendored third-party, globbed since nobody adds files here
file(GLOB_RECURSE FREERTOS_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/../Middlewares/Third_Party/FreeRTOS/Source/*.c"
)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${FREERTOS_SOURCES})

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE
    ${TOOLCHAIN_LINK_LIBRARIES}
    # Add user defined libraries
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add the map file to the list of files to be removed with 'clean' target
set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES ADDITIONAL_CLEAN_FILES ${CMAKE_PROJECT_NAME}.map)

# Validate that STM32CubeMX code is compatible with C standard
if((CMAKE_C_STANDARD EQUAL 90) OR (CMAKE_C_STANDARD EQUAL 99))
    message(ERROR "Generated code requires C11 or higher")
endif()









