/////////////////////////////////////////////////////////////////////////////
// Battery
#define BATTERY_OP_VOLTAGE_MIN 12000
#define BATTERY_OP_VOLTAGE_NOM 14800
#define BATTERY_OP_VOLTAGE_MAX 16800

#define BATTERY_CELL_OP_VOLTAGE_MIN 3000
#define BATTERY_CELL_OP_VOLTAGE_NOM 3700
#define BATTERY_CELL_OP_VOLTAGE_MAX 4200

#define BATTERY_CELL_COUNT 4

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
     */
    uint16_t overall_health; 
    /**
     * The voltage of each individual cell in the battery. LiPo batteries provide each 
     * cell's voltage as a cumulative value. The actual voltage of each cell must be 
     * calculated by subtracting the previous cell's cumulative voltage. The voltages 
     * will get less accurate as you move further through the list (i.e. cell_voltage[3] 
     * is the least accurate).
     *
     * This value gives a direct representation of each cell's voltage in millivolts (mV) 
     * (i.e 3.7V is 3700)
     */
    uint16_t cell_voltage[BATTERY_CELL_COUNT];
    
    uint32_t timestamp; /**< Timestamp for when the measurement was taken */
    uint8_t valid; /**< Indicates if the measurement is valid */
    uint8_t sequence; /**< Sequence number of the measurement, used to ensure a measurment is actually new */
} TelemetryBattery_t;
