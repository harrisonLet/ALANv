/////////////////////////////////////////////////////////////////////////////////////////
// Battery sensor driver for reading cell voltages and pack voltage of the LiPo Battery.
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

#include "sensorBattery.h"
#include "objRing.h"
#include "telemetry.h"

#include "queue.h"
#include <stdint.h>

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

static ADC_HandleTypeDef adc_handler;

static QueueHandle_t queue_metadata;

static uint16_t metadata_raw_cell[SENSOR_BATTERY_WINDOW][BATTERY_CELL_COUNT];
static Ring_o   metadata_raw_cell_ring;
static uint32_t metadata_raw_cell_sum[BATTERY_CELL_COUNT];

static uint8_t sequence;

static bool getRawTaps(uint16_t* buff);
static uint16_t rawTapsToMv(uint16_t counts);

/////////////////////////////////////////////////////////////////////////////////////////

HardwareInit_e sensorBattery_hardwareInit(void)
{
    sensorBattery_printf("sensorBattery_hardwareInit start");

    // Initialize Clocks
    sensorBattery_printf("    - init hardware clocks");
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_ADC12_CLK_ENABLE();

    // Initialize the GPIOs
    sensorBattery_printf("    - init hardware GPIO pins");
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Initialize ADC
    sensorBattery_printf("    - init ADC (Analog to Digital Converter) hardware");
    adc_handler.Instance = ADC1;
    adc_handler.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
    adc_handler.Init.Resolution            = ADC_RESOLUTION_16B;
    adc_handler.Init.ScanConvMode          = ADC_SCAN_ENABLE;
    adc_handler.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    adc_handler.Init.LowPowerAutoWait      = DISABLE;
    adc_handler.Init.ContinuousConvMode    = DISABLE;
    adc_handler.Init.NbrOfConversion       = BATTERY_CELL_COUNT;
    adc_handler.Init.DiscontinuousConvMode = DISABLE;
    adc_handler.Init.NbrOfDiscConversion   = 1;
    adc_handler.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    adc_handler.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    adc_handler.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
    adc_handler.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;
    adc_handler.Init.LeftBitShift          = ADC_LEFTBITSHIFT_NONE;
    adc_handler.Init.OversamplingMode      = DISABLE;
    if (HAL_ADC_Init(&adc_handler) != HAL_OK) { Error_Handler(); }
    if (HAL_ADCEx_Calibration_Start(&adc_handler, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK) { Error_Handler(); }

    // Scan mode converts ranks 1..4 upward in a single start, one channel per rank,
    // so the sequencer configuration only needs to happen once here.
    sensorBattery_printf("    - init ADC configuration for ranked reads");
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
        if (HAL_ADC_ConfigChannel(&adc_handler, &sConfig) != HAL_OK) { Error_Handler(); }
    }

    sensorBattery_printf("sensorBattery_hardwareInit complete");
    return HARDWARE_INIT_SUCCESS;
}



RtosInit_e sensorBattery_rtosInit(void)
{
    sensorBattery_printf("sensorBattery_rtosInit start");

    // Initialize the Queues used to share data
    sensorBattery_printf("    - init shared RTOS structures");
    if ((queue_metadata = xQueueCreate(1, sizeof(MetadataBattery_t))) == NULL)
    {
        sensorBattery_printf("ERROR: Failed to create queue for metadata");
        return RTOS_INIT_FAIL;
    }

    // Initialize the windows used to integrate data
    sensorBattery_printf("    - init ring buffer for windowed sensor reading");
    objRing_init(&metadata_raw_cell_ring, metadata_raw_cell, sizeof(metadata_raw_cell[0]), SENSOR_BATTERY_WINDOW);
    for (uint16_t i = 0; i < BATTERY_CELL_COUNT; i++)
    {
        metadata_raw_cell_sum[i] = 0;
    }

    sensorBattery_printf("sensorBattery_rtosInit complete");
    return RTOS_INIT_SUCCESS;
}



void sensorBattery_handler(void *argument)
{
    sensorBattery_printf("sensorBattery_handler start. Poll: %u ms.", (1000 / SENSOR_BATTERY_FREQUENCY));
    vTaskDelay(pdMS_TO_TICKS(BATTERY_SETTLE_MS));
    TickType_t lastWake = xTaskGetTickCount();
    for (;;)
    {
        // Read Frequency
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(1000 / SENSOR_BATTERY_FREQUENCY));

        // Metadata
        MetadataBattery_t metadata = {0};
        metadata.valid = VALIDITY_UNKNOWN;
        metadata.sequence = sequence++;
        metadata.timestamp = HAL_GetTick();

        uint16_t raw_cell[BATTERY_CELL_COUNT];
        if (!getRawTaps(raw_cell)) 
        {
            sensorBattery_printf("ERROR: Failed to read raw taps from battery ADC");
            metadata.valid = VALIDITY_ERROR_UNREACHABLE;
        }
        else 
        {
            for (uint32_t i = 0; i < BATTERY_CELL_COUNT; i++)
            {
                metadata.raw_cell[i] = raw_cell[i];
                if (raw_cell[i] == 0)     { metadata.valid = VALIDITY_UNTRUSTED_VALUE_IMPLAUSIBLE; break; }
                metadata.valid = VALIDITY_VALID;
            }

            if (metadata.valid == VALIDITY_VALID)
            {
                for (uint16_t i = 0; i < BATTERY_CELL_COUNT; i++) {
                    metadata_raw_cell_sum[i] += metadata.raw_cell[i];
                }
                uint16_t metadata_raw_cell_old[BATTERY_CELL_COUNT];
                if (objRing_push(&metadata_raw_cell_ring, &metadata.raw_cell, &metadata_raw_cell_old)) {
                    for (uint16_t i = 0; i < BATTERY_CELL_COUNT; i++) {
                        metadata_raw_cell_sum[i] -= metadata_raw_cell_old[i];
                    }
                }
            }
            else { }
        }
        #ifdef CM7_SENSOR_BATTERY_DEBUG
        char metadata_str[METADATA_BATTERY_STR_SIZE];
        sensorBattery_printf("%s", MetadataBattery_toString(&metadata, metadata_str));
        #endif
        xQueueOverwrite(queue_metadata, &metadata);

        // Telemetry
        if (metadata_raw_cell_ring.count == 0) { continue; }
        TelemetryBattery_t telemetry;
        telemetry.battery_voltage = rawTapsToMv(metadata_raw_cell_sum[BATTERY_CELL_COUNT - 1] / metadata_raw_cell_ring.count);
        uint16_t metadata_raw_cell_prev = 0;
        for (uint16_t i = 0; i < BATTERY_CELL_COUNT; i++) {
            uint16_t metadata_raw_cell_curr = metadata_raw_cell_sum[i] / metadata_raw_cell_ring.count;
            telemetry.cell_voltage[i] = rawTapsToMv(metadata_raw_cell_curr - metadata_raw_cell_prev);
            metadata_raw_cell_prev = metadata_raw_cell_curr;
        }
        telemetry_setBattery(&telemetry);
    }
}

static bool getRawTaps(uint16_t* buff)
{
    uint32_t sums[BATTERY_CELL_COUNT] = {0};
    for (uint32_t s = 0; s < BATTERY_AVG_SAMPLES; s++)
    {
        HAL_ADC_Start(&adc_handler);
        for (uint32_t i = 0; i < BATTERY_CELL_COUNT; i++)
        {
            if (HAL_ADC_PollForConversion(&adc_handler, BATTERY_ADC_TIMEOUT) != HAL_OK)  
            { 
                HAL_ADC_Stop(&adc_handler);
                return false; 
            }
            sums[i] += HAL_ADC_GetValue(&adc_handler);
        }
        HAL_ADC_Stop(&adc_handler);
    }

    for (uint32_t i = 0; i < BATTERY_CELL_COUNT; i++)
    {
        buff[i] = (uint16_t)(sums[i] / BATTERY_AVG_SAMPLES);
    }

    return true;
}

static uint16_t rawTapsToMv(uint16_t counts)
{
    uint32_t pin_mv = ((uint32_t)counts * BATTERY_ADC_VREF_MV) / BATTERY_ADC_FULL_SCALE;
    return (uint16_t)((pin_mv * BATTERY_DIVIDER_NUM) / BATTERY_DIVIDER_DEN);
}

/////////////////////////////////////////////////////////////////////////////////////////

bool sensorBattery_getMetadata(MetadataBattery_t *metadata)    { return xQueuePeek(queue_metadata, metadata, 0) == pdTRUE; }
