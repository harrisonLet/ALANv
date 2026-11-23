#ifndef LORA_TELEMETRY_H
#define LORA_TELEMETRY_H

#include <stdint.h>
#include "sharedBattery.h"

typedef enum {
    LORA_TELEMETRY_INVALID          = 0x0, // 0b00
    LORA_TELEMETRY_METADATA_VALID   = 0x1, // 0b01
    LORA_TELEMETRY_TELEMETRY_VALID  = 0x2, // 0b10
    LORA_TELEMETRY_ALL_VALID        = 0x3  // 0b11
} LoRaTelemetryValidity_e;

/*
 * Telemetry record sent from the boat to the ground station over LoRa.
 * Packed so the layout on the wire matches the struct exactly (80 bytes).
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

    /* GPS. Sized to what the GGA sentence gives (lat/lon to 1.67e-6 deg, altitude to 0.1 m), so nothing is lost */
    int32_t  latitude;               /* 1e-7 degrees (LORA_GPS_DEG_SCALE), + = north */
    int32_t  longitude;              /* 1e-7 degrees (LORA_GPS_DEG_SCALE), + = east */
    int16_t  altitude;               /* dm */
    uint8_t  sats_fix;               /* LORA_GPS_SATS_FIX: bits 0-4 satellites used in fix, bits 5-7 GGA fix quality (0 = no fix) */
    uint8_t  utc_hours;
    uint8_t  utc_minutes;
    uint8_t  utc_seconds;
    uint8_t  gps_valid;              /* Validity_e */

    /* GPS velocity, from the RMC sentence (Doppler, not differenced positions). Over ground, not through the water */
    uint16_t sog;                    /* speed over ground, cm/s (LORA_GPS_SOG_SCALE), LORA_GPS_VEL_NONE = no valid RMC in SENSOR_GPS_STALE */
    uint16_t cog;                    /* course over ground, 0..359 deg true, LORA_GPS_VEL_NONE = none (the GPS gives none when stopped) */

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
    uint16_t desired_heading;        /* 0..359 deg, heading PID target (set by LTP in AUTO), 0xFFFF = no target */
    uint8_t  route_segment;          /* leg LTP is sailing, 0 = to the first waypoint, 0xFF = none */
    int16_t  drift_distance;         /* cross-track distance from the segment, m, + = starboard of path */
    uint16_t last_cmd_seq;           /* sequence number of the last command received */
    uint32_t uptime_ms;              /* board uptime */
    uint8_t  last_cmd_result;        /* cmd_result_t of the command in last_cmd_seq */

    /* LoRa receive stats. The CM7 leaves these 0, the CM4 fills them in right before sending */
    uint8_t  rx_last_seq;            /* seq of the last packet received, heartbeats included */
    uint8_t  rx_last_type;           /* command type of that packet, 0x00 = heartbeat */
    int16_t  rx_last_rssi;           /* dBm, signal strength of that packet */
    int8_t   rx_last_snr;            /* quarter dB (LORA_RX_SNR_SCALE), SNR of that packet */
    uint8_t  rx_age_s;               /* seconds since anything was received, 255 = 255+ or never */
    int16_t  rx_noise;               /* dBm, channel RSSI with the radio listening, sampled right before this send */
    uint16_t rx_ok;                  /* packets received with a good CRC, wraps */
    uint16_t rx_crc_err;             /* packets received with a bad CRC, wraps */
    uint8_t  rx_rejected;            /* good CRC but failed the header/address/length check, wraps */
    uint8_t  rx_dropped;             /* valid command that could not be queued for the CM7, wraps */

} LoRaTelemetry_t;

/* The ground station decodes this exact layout, so a size change has to go out on both sides */
_Static_assert(sizeof(LoRaTelemetry_t) == 80, "LoRaTelemetry_t must be 80 bytes, update the ground station with it");

/* LoRa receive stats encoding */
#define LORA_RX_SNR_SCALE  4.0          /* rx_last_snr: dB * 4, as the radio reports it */
#define LORA_RX_AGE_NEVER  255U         /* rx_age_s when nothing has been received, or 255 s or more ago */

/* GPS field encoding */
#define LORA_GPS_DEG_SCALE 10000000.0   /* latitude/longitude: degrees * 1e7, about 1 cm */
#define LORA_GPS_ALT_SCALE 10.0         /* altitude: metres * 10 */
#define LORA_GPS_SATS_FIX(sats, fix) ((uint8_t)((((fix) > 7U ? 7U : (fix)) << 5) | ((sats) > 31U ? 31U : (sats))))
#define LORA_GPS_SATS(sats_fix)      ((uint8_t)((sats_fix) & 0x1FU))
#define LORA_GPS_FIX(sats_fix)       ((uint8_t)((sats_fix) >> 5))
#define LORA_GPS_SOG_SCALE 100.0        /* sog: m/s * 100 */
#define LORA_GPS_VEL_NONE  0xFFFFU      /* sog or cog unknown */

#endif
