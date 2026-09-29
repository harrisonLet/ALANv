# Shared driver/middleware sources and include dirs for the CM4 and CM7 cores.
# Included by CM4/CMakeLists.txt and CM7/CMakeLists.txt so new driver/BSP files
# just need to be dropped into these trees - no CMake edits required.

set(SAILBOAT_ROOT_DIR ${CMAKE_CURRENT_LIST_DIR})

file(GLOB_RECURSE COMMON_DRIVER_SOURCES CONFIGURE_DEPENDS
    "${SAILBOAT_ROOT_DIR}/Drivers/STM32H7xx_HAL_Driver/Src/*.c"
    "${SAILBOAT_ROOT_DIR}/Drivers/BSP/STM32H7xx_Nucleo/*.c"
)
list(FILTER COMMON_DRIVER_SOURCES EXCLUDE REGEX "_template\\.c$")

set(COMMON_INCLUDE_DIRS
    ${SAILBOAT_ROOT_DIR}/Drivers/STM32H7xx_HAL_Driver/Inc
    ${SAILBOAT_ROOT_DIR}/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy
    ${SAILBOAT_ROOT_DIR}/Drivers/BSP/STM32H7xx_Nucleo
    ${SAILBOAT_ROOT_DIR}/Drivers/CMSIS/Device/ST/STM32H7xx/Include
    ${SAILBOAT_ROOT_DIR}/Drivers/CMSIS/Include
)
