#pragma once

#include <stdint.h>

/////////////////////////////////////////////////////////////////////////////
// Battery
#define BATTERY_OP_VOLTAGE_MIN 12000u
#define BATTERY_OP_VOLTAGE_WARNING_LOW 13500u
#define BATTERY_OP_VOLTAGE_NOM 14800u
#define BATTERY_OP_VOLTAGE_WARNING_HIGH 16000u
#define BATTERY_OP_VOLTAGE_MAX 16800u

#define BATTERY_CELL_OP_VOLTAGE_MIN 3000u
#define BATTERY_CELL_OP_VOLTAGE_WARNING_LOW 3300u
#define BATTERY_CELL_OP_VOLTAGE_NOM 3700u
#define BATTERY_CELL_OP_VOLTAGE_WARNING_HIGH 4000u
#define BATTERY_CELL_OP_VOLTAGE_MAX 4200u

#define BATTERY_CELL_COUNT 4u

/**
 * Represents the current battery level of each cell in the battery and overall battery health.
 * - Safe Operating Range: 12V - 16.8V (3.0V - 4.2V per cell)
 * - Nominal Voltage: 14.8V (3.7V per cell)
 * 
 * SENSOR: Battery Monitor / ADC GPIO Pins
 */
typedef struct {
    /**
     * Overall voltage differential of the battery. This value gives a direct representation 
     * in millivolts (mV) (i.e 14.8V is 14800)
     *
     * - Operational Range: 12000 to 16800 mV
     * - Precision: 1 mV
     */
    uint16_t battery_voltage; 
    /**
     * The voltage of each individual cell in the battery. LiPo batteries provide each 
     * cell's voltage as a cumulative value. The actual voltage of each cell must be 
     * calculated by subtracting the previous cell's cumulative voltage. The voltages 
     * will get less accurate as you move further through the list (i.e. cell_voltage[3] 
     * is the least accurate).
     *
     * This value gives a direct representation of each cell's voltage in millivolts (mV) 
     * (i.e 3.7V is 3700)
     *
     * - Operational Range: 3000 to 4200 mV
     * - Precision: 1 mV
     */
    uint16_t cell_voltage[BATTERY_CELL_COUNT];
} TelemetryBattery_t;

#define TELEMETRY_BATTERY_STR_SIZE 64u
char *TelemetryBattery_toString(const TelemetryBattery_t *telemetry, char buf[TELEMETRY_BATTERY_STR_SIZE]);
