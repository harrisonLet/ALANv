/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

#include "sensorWind.h"
#include "objRing.h"
#include "sharedSensor.h"
#include "telemetry.h"

#include "queue.h"

/////////////////////////////////////////////////////////////////////////////////////////

#define WIND_ADDR 0x02

#define WIND_COUNTS     3600
#define WIND_HALF_TURN  1800
#define WIND_FORWARD    1800

#define ANG_WRAP(angle) (int16_t)(((((angle) + 180) % 360 + 360) % 360) - 180)

/////////////////////////////////////////////////////////////////////////////////////////

TaskHandle_t sensorWind_task;

/////////////////////////////////////////////////////////////////////////////////////////

static UART_HandleTypeDef uart_handler;

static QueueHandle_t metadata_queue;

static int32_t ang_unwrapped;
static int32_t ang_unwrapped_storage[SENSOR_WIND_WINDOW];
static Ring_o  ang_unwrapped_ring;
static int32_t ang_unwrapped_sum;

static uint16_t metadata_angle_prev;

static uint8_t metadata_sequence;

static HAL_StatusTypeDef wind_read(uint16_t *angle);
static uint16_t wind_crc16(const uint8_t *buf, int len);

/////////////////////////////////////////////////////////////////////////////////////////

HardwareInit_e sensorWind_hardwareInit(void)
{
    sensorWind_printf("sensorWind_hardwareInit start");

    // Initialize hardware clocks
    sensorWind_printf("    - init hardware clocks");
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_UART4_CLK_ENABLE();

    // Initialize GPIOs
    sensorWind_printf("    - init hardware GPIO pins");
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin       = GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // Initialize UART4 peripheral, PC10 as Tx, PC11 as Rx
    sensorWind_printf("    - init uart communication peripheral");
    // --
    // The below code forces the peripheral clock to use the D2PCLK1 source for UART2,
    // UART3, UART4, UART5, UART7, and UART8 peripherals. It shouldn't be necessary,
    // since D2PCLK1 is the default clock source for these UART peripherals, but I've
    // included the code incase it becomes a problem in the future.
    //
    // RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
    // PeriphClkInit.PeriphClockSelection       = RCC_PERIPHCLK_USART234578;
    // PeriphClkInit.Usart234578ClockSelection  = RCC_USART234578CLKSOURCE_D2PCLK1;
    // if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) { Error_Handler(); }
    // --
    uart_handler.Instance          = UART4;
    uart_handler.Init.BaudRate     = 9600;
    uart_handler.Init.WordLength   = UART_WORDLENGTH_8B;
    uart_handler.Init.StopBits     = UART_STOPBITS_1;
    uart_handler.Init.Parity       = UART_PARITY_NONE;
    uart_handler.Init.Mode         = UART_MODE_TX_RX;
    uart_handler.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    uart_handler.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&uart_handler) != HAL_OK) { Error_Handler(); }

    sensorWind_printf("    - UART4 initialized: SYSCLK:%lu, APB1:%lu, expected baud rate: 9600", 
        HAL_RCC_GetSysClockFreq(), HAL_RCC_GetPCLK1Freq());

    sensorWind_printf("sensorWind_hardwareInit complete");
    return HARDWARE_INIT_SUCCESS;
}



RtosInit_e sensorWind_rtosInit(void)
{
    sensorWind_printf("sensorWind_rtosInit start");

    // Initialize the Queues used to share data
    sensorWind_printf("    - init shared RTOS structures");
    if ((metadata_queue = xQueueCreate(1, sizeof(MetadataWind_t))) == NULL)
    {
        sensorWind_printf("ERROR: Failed to create queue for metadata");
        return RTOS_INIT_FAIL;
    }

    // Initialize the windows used to integrate data
    sensorWind_printf("    - init ring buffer for windowed sensor reading");
    objRing_init(&ang_unwrapped_ring, ang_unwrapped_storage, sizeof(int32_t), SENSOR_WIND_WINDOW);
    ang_unwrapped_sum = 0;
    ang_unwrapped = 0;
    metadata_angle_prev = WIND_FORWARD;

    sensorWind_printf("sensorWind_rtosInit complete");
    return RTOS_INIT_SUCCESS;
}



void sensorWind_handler(void *argument)
{
    sensorWind_printf("sensorWind_handler start");

    TickType_t lastWake = xTaskGetTickCount();
    for(;;)
    {
        // Read Frequency
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(1000 / SENSOR_WIND_FREQUENCY));

        // Metdata
        MetadataWind_t metadata = {0};
        metadata.valid = VALIDITY_UNKNOWN;
        metadata.sequence = metadata_sequence++;
        metadata.timestamp = HAL_GetTick();

        HAL_StatusTypeDef status = wind_read(&metadata.angle);
        if (status != HAL_OK)
        {
            sensorWind_printf("ERROR: Failed to read wind sensor");
            metadata.valid = VALIDITY_ERROR_UNREACHABLE;
        }
        else 
        {
            if (metadata.angle >= WIND_COUNTS) { metadata.valid = VALIDITY_UNTRUSTED_VALUE_OUT_OF_RANGE; }
            else { metadata.valid = VALIDITY_VALID; }

            if (metadata.valid == VALIDITY_VALID)
            {
                int32_t delta = (int32_t)metadata.angle - metadata_angle_prev;
                if (delta >  WIND_HALF_TURN) { delta -= WIND_COUNTS; }
                if (delta < -WIND_HALF_TURN) { delta += WIND_COUNTS; }
                ang_unwrapped += delta;
                metadata_angle_prev = metadata.angle;

                int32_t ang_unwrapped_old;
                ang_unwrapped_sum += ang_unwrapped;
                if (objRing_push(&ang_unwrapped_ring, &ang_unwrapped, &ang_unwrapped_old)) {
                    ang_unwrapped_sum -= ang_unwrapped_old;
                }
            }
            else {}
        }
        #ifdef CM7_SENSOR_WIND_DEBUG
        char metadata_str[METADATA_WIND_STR_SIZE];
        sensorWind_printf("%s", MetadataWind_toString(&metadata, metadata_str));
        #endif
        xQueueOverwrite(metadata_queue, &metadata);

        // Telemetry
        if (ang_unwrapped_ring.count == 0 || metadata.valid != VALIDITY_VALID) { continue; }
        TelemetryWind_t telemetry;
        int32_t degrees = ((ang_unwrapped_sum / (int32_t)ang_unwrapped_ring.count) * 360) / WIND_COUNTS;
        telemetry.ang_wind = ANG_WRAP(degrees);
        telemetry.ang_wind_r = ANG_WRAP(180 - telemetry.ang_wind);
        telemetry_setWind(&telemetry);
    }
}



static HAL_StatusTypeDef wind_read(uint16_t *angle)
{
    uint8_t cmd[8] = {WIND_ADDR, 0x03, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00};
    uint16_t crc = wind_crc16(cmd, 6);
    cmd[6] = crc & 0xFF;
    cmd[7] = (crc >> 8) & 0xFF;
    
    // Clear overrun flag and flush the RX data register
    __HAL_UART_CLEAR_OREFLAG(&uart_handler);
    __HAL_UART_FLUSH_DRREGISTER(&uart_handler);

    // Transmit command to wind sensor
    if (HAL_UART_Transmit(&uart_handler, cmd, 8, 30) != HAL_OK) 
    {
        sensorWind_printf("ERROR: Failed to transmit wind sensor command");
        return HAL_ERROR;
    }

    // Read response (7 bytes: addr, func (0x03), byte count (0x02), data_hi, data_lo, crc_lo, crc_hi)
    uint8_t resp[7];
    if (HAL_UART_Receive(&uart_handler, resp, 7, 30) != HAL_OK) {
        sensorWind_printf("ERROR: Failed to read wind sensor response");
        return HAL_ERROR;
    }

    // Verify
    if (resp[0] != WIND_ADDR || resp[1] != 0x03 || resp[2] != 0x02) {
        sensorWind_printf("ERROR: Invalid wind sensor response header");
        return HAL_ERROR;
    }
    uint16_t crc_calc = wind_crc16(resp, 5);
    uint16_t crc_recv = (uint16_t)resp[5] | ((uint16_t)resp[6] << 8);
    if (crc_calc != crc_recv) {
        sensorWind_printf("ERROR: CRC mismatch in wind sensor response");
        return HAL_ERROR;
    }

    // Extract angle
    *angle = ((uint16_t)resp[3] << 8) | (uint16_t)resp[4];
    return HAL_OK;
}

static uint16_t wind_crc16(const uint8_t *buf, int len)
{
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < len; i++)
    {
        crc ^= (uint16_t)buf[i];
        for (int b = 8; b != 0; b--)
        {
            if (crc & 0x0001) { crc >>= 1; crc ^= 0xA001; }
            else              { crc >>= 1; }
        }
    }
    return crc;
}


/////////////////////////////////////////////////////////////////////////////////////////