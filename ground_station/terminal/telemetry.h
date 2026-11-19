#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>

/*
 * Telemetry record sent from the boat to the ground station over LoRa.
 * Packed so the layout on the wire matches the struct exactly (62 bytes).
 *
 * Copy of Common/Inc/loraTelemetry.h on command-pipeline. Keep the two identical.
 */
typedef struct __attribute__((packed)) {

    /* Battery */
    uint16_t battery_voltage;        /* whole pack, mV */
    uint16_t cell_voltage[4];        /* each cell, mV */
    uint8_t  battery_valid;          /* Validity_e */

    /* Main sail encoder (AS5600) */
    int16_t  sail_ang_forward;       /* sail vs hull centerline, -180..180 deg, + = starboard */
    uint16_t sail_encoder_magnitude; /* AS5600 CORDIC magnitude */
    uint8_t  sail_encoder_status;    /* AS5600 MD/ML/MH bits */
    uint8_t  encoder_valid;          /* Validity_e */

    /* Wind vane */
    int16_t  ang_wind;               /* apparent wind vs bow, -180..180 deg, + = from starboard */
    uint8_t  wind_valid;             /* Validity_e */

    /* GPS. Sized to what the GGA sentence gives (lat/lon to 1.67e-6 deg, altitude to 0.1 m), so nothing is lost */
    int32_t  latitude;               /* 1e-7 degrees (LORA_GPS_DEG_SCALE), + = north */
    int32_t  longitude;              /* 1e-7 degrees (LORA_GPS_DEG_SCALE), + = east */
    int16_t  altitude;               /* dm */
    uint8_t  sats_fix;               /* LORA_GPS_SATS_FIX: bits 0-4 satellites used in fix, bits 5-7 GGA fix quality (0 = no fix) */
    uint8_t  utc_hours;
    uint8_t  utc_minutes;
    uint8_t  utc_seconds;
    uint8_t  gps_valid;              /* Validity_e */

    /* IMU (BNO055) */
    uint8_t  imu_cal;                /* calibration: sys/gyro/acc/mag, 2 bits each */
    int8_t   imu_temp;               /* deg C */
    uint8_t  imu_valid;              /* Validity_e */

    /* Fused orientation */
    uint16_t heading;                /* 0..359 deg, true north */
    int16_t  roll;                   /* -180..180 deg, + = starboard */
    int16_t  pitch;                  /* -180..180 deg, + = bow */
    uint8_t  orientation_valid;      /* Validity_e */

    /* Servos, as currently driven by the STM (after clamping), not as commanded */
    int16_t  rudder_servo_angle;     /* -135..135 deg */
    int16_t  sail_servo_angle;       /* -135..135 deg */

    /* Control */
    uint8_t  mode;                   /* system_mode_t (commands.h) */
    uint16_t desired_heading;        /* 0..359 deg, heading PID target (LTP later), 0xFFFF = no target */
    uint8_t  route_segment;          /* index of the active nautical vector */
    int16_t  drift_distance;         /* cross-track distance from the segment, m, + = starboard of path */
    uint16_t last_cmd_seq;           /* seq of the last non-heartbeat command the boat executed */
    uint32_t uptime_ms;              /* board uptime */
    uint8_t  last_cmd_result;        /* cmd_result_t of the command in last_cmd_seq */

} LoRaTelemetry_t;

/* GPS field encoding */
#define LORA_GPS_DEG_SCALE 10000000.0   /* latitude/longitude: degrees * 1e7, about 1 cm */
#define LORA_GPS_ALT_SCALE 10.0         /* altitude: metres * 10 */
#define LORA_GPS_SATS_FIX(sats, fix) ((uint8_t)((((fix) > 7U ? 7U : (fix)) << 5) | ((sats) > 31U ? 31U : (sats))))
#define LORA_GPS_SATS(sats_fix)      ((uint8_t)((sats_fix) & 0x1FU))
#define LORA_GPS_FIX(sats_fix)       ((uint8_t)((sats_fix) >> 5))

#endif
