# Shared driver/middleware sources and include dirs for the CM4 and CM7 cores.
# Included by CM4/CMakeLists.txt and CM7/CMakeLists.txt so new driver/BSP files
# just need to be dropped into these trees - no CMake edits required.

set(SAILBOAT_ROOT_DIR ${CMAKE_CURRENT_LIST_DIR})

file(GLOB_RECURSE COMMON_DRIVER_SOURCES CONFIGURE_DEPENDS
    "${SAILBOAT_ROOT_DIR}/Drivers/STM32H7xx_HAL_Driver/Src/*.c"
    "${SAILBOAT_ROOT_DIR}/Drivers/BSP/STM32H7xx_Nucleo/*.c"
)
list(FILTER COMMON_DRIVER_SOURCES EXCLUDE REGEX "_template\\.c$")

# Shared app code (e.g. to_string helpers for the shared structs). The system
# file is already listed explicitly in each core's CMakeLists, so skip it here.
file(GLOB COMMON_APP_SOURCES CONFIGURE_DEPENDS "${SAILBOAT_ROOT_DIR}/Common/Src/*.c")
list(FILTER COMMON_APP_SOURCES EXCLUDE REGEX "system_stm32h7xx")

set(COMMON_INCLUDE_DIRS
    ${SAILBOAT_ROOT_DIR}/Common/Inc
    ${SAILBOAT_ROOT_DIR}/Drivers/STM32H7xx_HAL_Driver/Inc
    ${SAILBOAT_ROOT_DIR}/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy
    ${SAILBOAT_ROOT_DIR}/Drivers/BSP/STM32H7xx_Nucleo
    ${SAILBOAT_ROOT_DIR}/Drivers/CMSIS/Device/ST/STM32H7xx/Include
    ${SAILBOAT_ROOT_DIR}/Drivers/CMSIS/Include
)

# Newlib-nano ships printf/scanf without floating point support by default.
# Force the float variants in for both cores (gcc / newlib toolchains only).
set(COMMON_LINK_OPTIONS "")
if(CMAKE_C_COMPILER_ID STREQUAL "GNU")
    # -u and its symbol must be a single option, CMake de-duplicates repeated "-u" entries
    list(APPEND COMMON_LINK_OPTIONS -Wl,-u,_printf_float -Wl,-u,_scanf_float)
endif()
