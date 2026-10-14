/////////////////////////////////////////////////////////////////////////////////////////
// Battery sensor driver for reading cell voltages and pack voltage of the LiPo Battery
//
// GPIO:          PA0  PA4  PA5  PA6
// ADC1 channels: 16   18   19   3
// Battery Cell:  BC1  BC2  BC3  BC4
//
// Cell voltages are measured cumulatively from the pack negative terminal, ie BC1=cell1,
// BC2=cell1+cell2 and so on until BC4=full pack voltage.
// We chose not to use the PA1 pin because it is tied to the Nucleo's on-board Ethernet 
// PHY REF_CLK, which injects noise regardless of firmware ETH usage.
/////////////////////////////////////////////////////////////////////////////////////////

#include "system.h"

#include "sensorBattery.h"
#include "objRing.h"

#include "queue.h"

#include "sharedSensor.h"
#include "sharedBattery.h"

/////////////////////////////////////////////////////////////////////////////////////////

// RC filter tau = (82k || 18k) * 100nF ~= 1.48ms; 10ms is ~6.8 tau (>99.8% settled)
#define BATTERY_SETTLE_MS           10u
#define BATTERY_ADC_TIMEOUT         10u
// Number of raw conversions averaged per channel each cycle (integer average, no floats)
#define BATTERY_AVG_SAMPLES         8u

// Fixed-point conversion constants (no floating point)
#define BATTERY_ADC_VREF_MV         3300u
#define BATTERY_ADC_FULL_SCALE      65535u
// Divider: Vtap = Vpin * (R_TOP+R_BOTTOM)/R_BOTTOM = Vpin * (82k+18k)/18k = Vpin * 100k/18k
#define BATTERY_DIVIDER_NUM         100u
#define BATTERY_DIVIDER_DEN         18u


/////////////////////////////////////////////////////////////////////////////////////////

TaskHandle_t sensorBattery_task;

/////////////////////////////////////////////////////////////////////////////////////////

static ADC_HandleTypeDef hadc1;

static QueueHandle_t queue_metadata;
static QueueHandle_t queue_telemetry;

static MetadataBattery_t storage_metadata[SENSOR_BATTERY_WINDOW];
static Ring_o ring_metadata;
static uint32_t sum_metadata_raw_cell[BATTERY_CELL_COUNT];
static uint32_t sum_metadata_voltage_cell[BATTERY_CELL_COUNT];

static uint8_t sequence;

static uint16_t* getRawTaps(void);
static uint16_t rawTapsToMv(uint16_t counts);

/////////////////////////////////////////////////////////////////////////////////////////

void sensorBattery_hardwareInit(void)
{
    // Initialize the memory constructs
    queue_metadata =  xQueueCreate(1, sizeof(MetadataBattery_t));
    queue_telemetry = xQueueCreate(1, sizeof(TelemetryBattery_t));

    objRing_init(&ring_metadata, storage_metadata, sizeof(MetadataBattery_t), SENSOR_BATTERY_WINDOW);
    for (uint16_t i = 0; i < BATTERY_CELL_COUNT; i++)
    {
        sum_metadata_raw_cell[i] = 0;
        sum_metadata_voltage_cell[i] = 0;
    }

    // Initialize the GPIOs
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Initialize ADC
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution            = ADC_RESOLUTION_16B;
    hadc1.Init.ScanConvMode          = ADC_SCAN_ENABLE;
    hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    hadc1.Init.LowPowerAutoWait      = DISABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;
    hadc1.Init.NbrOfConversion       = BATTERY_CELL_COUNT;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.NbrOfDiscConversion   = 1;
    hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
    hadc1.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;
    hadc1.Init.LeftBitShift          = ADC_LEFTBITSHIFT_NONE;
    hadc1.Init.OversamplingMode      = DISABLE;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) { Error_Handler(); }
    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK) { Error_Handler(); }

    // Scan mode converts ranks 1..4 upward in a single start, one channel per rank,
    // so the sequencer configuration only needs to happen once here.
    static const uint32_t battery_adcChannels[BATTERY_CELL_COUNT] =
    {
        ADC_CHANNEL_16, // PA0 - BC1 (Battery Cell 1)
        ADC_CHANNEL_18, // PA4 - BC2
        ADC_CHANNEL_19, // PA5 - BC3
        ADC_CHANNEL_3,  // PA6 - BC4
    };
    static const uint32_t ranks[BATTERY_CELL_COUNT] = { 
        ADC_REGULAR_RANK_1, 
        ADC_REGULAR_RANK_2, 
        ADC_REGULAR_RANK_3, 
        ADC_REGULAR_RANK_4 
    };
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.SamplingTime           = ADC_SAMPLETIME_64CYCLES_5;
    sConfig.SingleDiff             = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber           = ADC_OFFSET_NONE;
    sConfig.Offset                 = 0;
    sConfig.OffsetRightShift       = DISABLE;
    sConfig.OffsetSignedSaturation = DISABLE;
    for (uint32_t i = 0; i < BATTERY_CELL_COUNT; i++)
    {
        sConfig.Channel = battery_adcChannels[i];
        sConfig.Rank    = ranks[i];
        if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }
    }
}



void sensorBattery_handler(void *argument)
{
    vTaskDelay(pdMS_TO_TICKS(BATTERY_SETTLE_MS));
    for (;;)
    {
        // Read Frequency
        vTaskDelay(pdMS_TO_TICKS(1000 / SENSOR_BATTERY_FREQUENCY));

        // Metadata
        MetadataBattery_t metadata;
        uint16_t *rawTaps = getRawTaps();
        metadata.valid = VALIDITY_UNKNOWN;
        metadata.sequence = sequence++;
        for (uint32_t i = 0; i < BATTERY_CELL_COUNT; i++)
        {
            uint16_t mv = rawTapsToMv(rawTaps[i]);
            metadata.raw_cell[i] = rawTaps[i];
            metadata.voltage_cell[i] = mv;
            if (rawTaps[i] == 0)                  { metadata.valid = VALIDITY_UNTRUSTED_VALUE_IMPLAUSIBLE; break; }
            if (mv < BATTERY_CELL_OP_VOLTAGE_MIN) { metadata.valid = VALIDITY_UNTRUSTED_UNDER_SATURATION; break; }
            if (mv > BATTERY_CELL_OP_VOLTAGE_MAX) { metadata.valid = VALIDITY_UNTRUSTED_OVER_SATURATION; break; }
            metadata.valid = VALIDITY_VALID;
        }
        metadata.timestamp = HAL_GetTick();
        if (metadata.valid == VALIDITY_VALID)
        {
            for (uint16_t i = 0; i < BATTERY_CELL_COUNT; i++) {
                sum_metadata_raw_cell[i] += metadata.raw_cell[i];
                sum_metadata_voltage_cell[i] += metadata.voltage_cell[i];
            }
            MetadataBattery_t metadata_old;
            if (objRing_push(&ring_metadata, &metadata, &metadata_old)) {
                for (uint16_t i = 0; i < BATTERY_CELL_COUNT; i++) {
                    sum_metadata_raw_cell[i] -= metadata_old.raw_cell[i];
                    sum_metadata_voltage_cell[i] -= metadata_old.voltage_cell[i];
                }
            }
            xQueueOverwrite(queue_metadata, &metadata);
        }
        else { }

        // Telemetry
        TelemetryBattery_t telemetry;
        telemetry.battery_voltage = sum_metadata_voltage_cell[BATTERY_CELL_COUNT - 1] / SENSOR_BATTERY_WINDOW;
        uint16_t cell_prev = 0;
        for (uint16_t i = 0; i < BATTERY_CELL_COUNT; i++)
        {
            telemetry.cell_voltage[i] = (sum_metadata_voltage_cell[i] / SENSOR_BATTERY_WINDOW) - cell_prev;
            cell_prev = telemetry.cell_voltage[i];
        }
        xQueueOverwrite(queue_telemetry, &telemetry);
    }
}

static uint16_t* getRawTaps(void)
{
    uint16_t battery_rawTaps[BATTERY_CELL_COUNT];

    uint32_t sums[BATTERY_CELL_COUNT] = {0};
    for (uint32_t s = 0; s < BATTERY_AVG_SAMPLES; s++)
    {
        HAL_ADC_Start(&hadc1);
        for (uint32_t i = 0; i < BATTERY_CELL_COUNT; i++)
        {
            if (HAL_ADC_PollForConversion(&hadc1, BATTERY_ADC_TIMEOUT) == HAL_OK)  { sums[i] += HAL_ADC_GetValue(&hadc1); }
            else                                                                   { sums[i] += BATTERY_CELL_OP_VOLTAGE_NOM; }
        }
        HAL_ADC_Stop(&hadc1);
    }

    for (uint32_t i = 0; i < BATTERY_CELL_COUNT; i++)
    {
        battery_rawTaps[i] = (uint16_t)(sums[i] / BATTERY_AVG_SAMPLES);
    }

    return battery_rawTaps;
}

static uint16_t rawTapsToMv(uint16_t counts)
{
    uint32_t pin_mv = ((uint32_t)counts * BATTERY_ADC_VREF_MV) / BATTERY_ADC_FULL_SCALE;
    return (uint16_t)((pin_mv * BATTERY_DIVIDER_NUM) / BATTERY_DIVIDER_DEN);
}



bool sensorBattery_getTelemetry(TelemetryBattery_t *telemetry) { return xQueuePeek(queue_telemetry, telemetry, 0) == pdTRUE; }
bool sensorBattery_getMetadata(MetadataBattery_t *metadata)    { return xQueuePeek(queue_metadata, metadata, 0) == pdTRUE; }
