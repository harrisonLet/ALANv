#pragma once

/////////////////////////////////////////////////////////////////////////////////////////
// Shared Configuration
/////////////////////////////////////////////////////////////////////////////////////////
#define DUAL_CORE_BOOT_SYNC_SEQUENCE
#define IPC_ENABLE // Inter-Processor Communication (CM7 <-> CM4)


/////////////////////////////////////////////////////////////////////////////////////////
// CM4 Configuration
/////////////////////////////////////////////////////////////////////////////////////////
#define CM4_ENABLE
// #define CM4_LORA_COMMANDS_ENABLE // Ground -> boat commands. Off: radio only transmits telemetry, never listens
// #define CM4_LORA_DUMMY_TELEMETRY // Test: send a fixed telemetry packet every tick instead of reading SRAM4
#define CM4_FAKE_COMMANDS // Test: feed a scripted command list (fakeCommands.c) into the command parse path. Radio only sends telemetry. Turn CM4_LORA_COMMANDS_ENABLE off

#if defined(CM4_FAKE_COMMANDS) && defined(CM4_LORA_COMMANDS_ENABLE)
#error "Pick one command source: CM4_FAKE_COMMANDS or CM4_LORA_COMMANDS_ENABLE"
#endif



#ifdef CM4_ENABLE
// #define CM4_DEBUG
#endif


/////////////////////////////////////////////////////////////////////////////////////////
// CM7 Configuration
/////////////////////////////////////////////////////////////////////////////////////////
#define CM7_ENABLE
#define CM7_LIVE_ENABLE
#define CM7_COMMANDER_ENABLE
#define CM7_TELEMETRY_ENABLE
#define CM7_SENSOR_BATTERY_ENABLE
#define CM7_SENSOR_ENCODER_ENABLE
#define CM7_SENSOR_GPS_ENABLE
#define CM7_SENSOR_IMU_ENABLE
#define CM7_SENSOR_WIND_ENABLE
#define CM7_FUSION_ORIENTATION_ENABLE // This module requires data from the SENSOR_GPS and SENSOR_IMU tasks, it will not run if they are not also enabled
#define CM7_SERVO_RUDDER_ENABLE
#define CM7_ARBITER_RUDDER_ENABLE
#define CM7_PID_HEADING_ENABLE
#define CM7_SERVO_SAIL_ENABLE
#define CM7_ARBITER_TAIL_ENABLE
#define CM7_PID_SAIL_ENABLE
#define CM7_PROCESS_LSP_ENABLE
// #define CM7_CONSOLE_ENABLE


#ifdef CM7_ENABLE
#define CM7_DEBUG
#endif

#ifdef CM7_LIVE_ENABLE
// #define CM7_LIVE_DEBUG
// #define CM7_LIVE_DEBUG_STACK // CM7_LIVE_DEBUG must be enabled
#endif

#ifdef CM7_COMMANDER_ENABLE
#define CM7_COMMANDER_DEBUG
// #define COMMANDER_SELFTEST // Bench test: runs a scripted command list at boot. Needs CM7_COMMANDER_DEBUG to print. Leave CM7_CONSOLE_ENABLE off
#endif

#ifdef CM7_TELEMETRY_ENABLE
// #define CM7_TELEMETRY_DEBUG
#endif

#ifdef CM7_SENSOR_BATTERY_ENABLE
// #define CM7_SENSOR_BATTERY_DEBUG
#endif

#ifdef CM7_SENSOR_ENCODER_ENABLE
// #define CM7_SENSOR_ENCODER_DEBUG
#endif

#ifdef CM7_SENSOR_GPS_ENABLE
// #define CM7_SENSOR_GPS_DEBUG
#endif

#ifdef CM7_SENSOR_IMU_ENABLE
// #define CM7_SENSOR_IMU_DEBUG
#define CM7_SENSOR_IMU_PRE_CALIBRATED // If active the IMU will use the hard-coded calibration offsets
// #define CM7_SENSOR_IMU_FULL_DATA // If active the IMU will provide data for all individual sensors in additon to the fused data, if inactive it will provide only fused data
#endif

#ifdef CM7_SENSOR_WIND_ENABLE
// #define CM7_SENSOR_WIND_DEBUG
#endif

#ifdef CM7_FUSION_ORIENTATION_ENABLE
// #define CM7_FUSION_ORIENTATION_DEBUG
#endif

#ifdef CM7_ARBITER_RUDDER_ENABLE
// #define CM7_ARBITER_RUDDER_DEBUG
#endif

#ifdef CM7_PID_HEADING_ENABLE
#define CM7_PID_HEADING_DEBUG
#endif

#ifdef CM7_ARBITER_TAIL_ENABLE
// #define CM7_ARBITER_TAIL_DEBUG
#endif

#ifdef CM7_PID_SAIL_ENABLE
// #define CM7_PID_SAIL_DEBUG
#endif

#ifdef CM7_PROCESS_LSP_ENABLE
// #define CM7_PROCESS_LSP_DEBUG
#endif
