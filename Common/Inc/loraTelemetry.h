#ifndef LORA_TELEMETRY_H
#define LORA_TELEMETRY_H

#include <stdint.h>
#include "sharedBattery.h"

/*
 * Telemetry record sent from the boat to the ground station over LoRa.
 * Packed so the layout on the wire matches the struct exactly (72 bytes).
 */
typedef struct __attribute__((packed)) {

    /* Battery */
    uint16_t battery_voltage;        /* whole pack, mV */
    uint16_t cell_voltage[BATTERY_CELL_COUNT];        /* each cell, mV */
    uint8_t  battery_valid;          /* Validity_e */

    /* Main sail encoder (AS5600) */
    int16_t  sail_ang_forward;       /* sail vs hull centerline, -180..180 deg, + = starboard */
    uint16_t sail_encoder_magnitude; /* AS5600 CORDIC magnitude */
    uint8_t  sail_encoder_status;    /* AS5600 MD/ML/MH bits */
    uint8_t  encoder_valid;          /* Validity_e */

    /* Wind vane */
    int16_t  ang_wind;               /* apparent wind vs bow, -180..180 deg, + = from starboard */
    uint8_t  wind_valid;             /* Validity_e */

    /* GPS */
    double   latitude;               /* degrees, + = north */
    double   longitude;              /* degrees, + = east */
    float    altitude;               /* m */
    uint8_t  satellites;             /* satellites used in fix */
    uint8_t  fix_valid;              /* GGA fix quality, 0 = no fix */
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
    uint8_t  mode;                   /* current mode from the control task */
    uint16_t desired_heading;        /* 0..359 deg, from LTP */
    uint8_t  route_segment;          /* index of the active nautical vector */
    int16_t  drift_distance;         /* cross-track distance from the segment, m, + = starboard of path */
    uint16_t last_cmd_seq;           /* sequence number of the last command received */
    uint32_t uptime_ms;              /* board uptime */

} LoRaTelemetry_t;

#endif
