/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

#include "sensorGPS.h"

#include "main.h"
#include "projdefs.h"
#include "queue.h"
#include <stdint.h>

/////////////////////////////////////////////////////////////////////////////////////////

#define SENSOR_GPS_DMA_BUFFER_SIZE 256u
#define SENSOR_GPS_IRQ_PRIORITY 6u // must be >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY

/////////////////////////////////////////////////////////////////////////////////////////

TaskHandle_t sensorGPS_task;

/////////////////////////////////////////////////////////////////////////////////////////

static UART_HandleTypeDef uart_handler;
static DMA_HandleTypeDef uart_dma_handler;

// Circular DMA target. Must live in AXI SRAM since DMA1 cannot access DTCM.
// If the D-cache is ever enabled, this region must be made non-cacheable (MPU)
// or invalidated before reading.
static uint8_t uart_dma_buf[SENSOR_GPS_DMA_BUFFER_SIZE] __attribute__((section(".axi_sram"), aligned(32)));
static volatile uint16_t uart_dma_head; // next write position, updated in ISR
static uint16_t uart_dma_tail;          // next read position, owned by the consumer task
static uint8_t uart_dma_error;

static QueueHandle_t metadata_queue;

static char metadata_raw[128];
static uint8_t metadata_raw_head;
static double metadata_latitude;
static double metadata_longitude;
static float metadata_altitude;
static uint8_t metadata_satellites;
static uint8_t metadata_hours;
static uint8_t metadata_minutes;
static uint8_t metadata_seconds;
static uint8_t metadata_sequence;

static void uartDMAHandler_start(void);
static Validity_e interpretGGAStr(const char *metadata_raw);

/////////////////////////////////////////////////////////////////////////////////////////

HardwareInit_e sensorGPS_hardwareInit(void)
{
    sensorGPS_printf("sensorGPS_hardwareInit start");

    // Initialize hardware clocks
    sensorGPS_printf("    - init hardware clocks");
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_UART7_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    // Initialize GPIOs, using PF6 as RX and PF7 as TX
    sensorGPS_printf("    - init hardware GPIO pins");
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin       = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_UART7;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    // Initialize UART7 peripheral
    sensorGPS_printf("    - init uart communication peripheral");
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
    uart_handler.Instance          = UART7;
    uart_handler.Init.BaudRate     = 9600;
    uart_handler.Init.WordLength   = UART_WORDLENGTH_8B;
    uart_handler.Init.StopBits     = UART_STOPBITS_1;
    uart_handler.Init.Parity       = UART_PARITY_NONE;
    uart_handler.Init.Mode         = UART_MODE_TX_RX;
    uart_handler.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    uart_handler.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&uart_handler) != HAL_OK) { Error_Handler(); }

    // Initialize DMA1 stream 0 as a circular UART7 RX channel
    sensorGPS_printf("    - init dma receive channel");
    uart_dma_handler.Instance                 = DMA1_Stream0;
    uart_dma_handler.Init.Request             = DMA_REQUEST_UART7_RX;
    uart_dma_handler.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    uart_dma_handler.Init.PeriphInc           = DMA_PINC_DISABLE;
    uart_dma_handler.Init.MemInc              = DMA_MINC_ENABLE;
    uart_dma_handler.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    uart_dma_handler.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    uart_dma_handler.Init.Mode                = DMA_CIRCULAR;
    uart_dma_handler.Init.Priority            = DMA_PRIORITY_LOW;
    uart_dma_handler.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&uart_dma_handler) != HAL_OK) { Error_Handler(); }
    __HAL_LINKDMA(&uart_handler, hdmarx, uart_dma_handler);

    HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, SENSOR_GPS_IRQ_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
    HAL_NVIC_SetPriority(UART7_IRQn, SENSOR_GPS_IRQ_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(UART7_IRQn);
    sensorGPS_printf("    - UART7 initialized: SYSCLK:%lu, APB1:%lu, expected baud rate: 9600", 
        HAL_RCC_GetSysClockFreq(), HAL_RCC_GetPCLK1Freq());

    sensorGPS_printf("sensorGPS_hardwareInit complete");
    return HARDWARE_INIT_SUCCESS;
}



RtosInit_e sensorGPS_rtosInit(void)
{
    sensorGPS_printf("sensorGPS_rtosInit start");

    // Initialize the Queues used to share data
    sensorGPS_printf("    - init shared RTOS structures");
    if ((metadata_queue = xQueueCreate(1, sizeof(MetadataGPS_t))) == NULL)
    {
        sensorGPS_printf("ERROR: Failed to create queue for metadata");
        return RTOS_INIT_FAIL;
    }
    MetadataGPS_t metadata = {0};
    metadata.valid = VALIDITY_VALID_WARMING;
    metadata.sequence = metadata_sequence++;
    metadata.timestamp = HAL_GetTick();
    xQueueOverwrite(metadata_queue, &metadata);

    sensorGPS_printf("sensorGPS_rtosInit complete");
    return RTOS_INIT_SUCCESS;
}



void sensorGPS_handler(void *argument)
{
    sensorGPS_printf("sensorGPS_handler start. Poll: %u ms", 1000 / SENSOR_GPS_FREQUENCY);
    vTaskDelay(pdMS_TO_TICKS(5000)); // 5 seconds delay for the GPS module to stabilize
    uartDMAHandler_start(); // start UART DMA reception after GPS module stabilizes
    for (;;)
    {
        // Error handling for UART DMA
        if (uart_dma_error) {
            sensorGPS_printf("UART DMA error detected: %u", uart_dma_error);
            uartDMAHandler_start();
            uart_dma_error = 0; // Clear the error flag after reporting
        }
        
        // Metadata
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(SENSOR_GPS_STALE)) == 0)
        {
            MetadataGPS_t metadata = {0};
            metadata.valid = VALIDITY_INVALID_GPS_STALE;
            metadata.sequence = metadata_sequence++;
            metadata.timestamp = HAL_GetTick();
            xQueueOverwrite(metadata_queue, &metadata);
        }
        else
        {
            uint16_t head = uart_dma_head;
            while (uart_dma_tail != head)
            {
                char c = uart_dma_buf[uart_dma_tail];
                uart_dma_tail = (uart_dma_tail + 1) % SENSOR_GPS_DMA_BUFFER_SIZE;
                
                if (c == '$') 
                {
                    if (metadata_raw_head != 0) 
                    { 
                        metadata_raw[metadata_raw_head] = '\0'; // Null-terminate the incomplete sentence
                        sensorGPS_printf("Incomplete GPS sentence before new $, %s", metadata_raw); 
                    }
                    metadata_raw_head = 0; // Reset the raw GPS sentence buffer for the new sentence
                    metadata_raw[metadata_raw_head++] = c; // Start the new GPS sentence with '$'
                }
                else if (c == '\n') 
                {
                    if (metadata_raw_head != 0 && metadata_raw[metadata_raw_head - 1] == '\r') { metadata_raw_head--; }
                    metadata_raw[metadata_raw_head] = '\0';

                    MetadataGPS_t metadata = {0};
                    metadata.valid = VALIDITY_UNKNOWN;
                    metadata.sequence = metadata_sequence++;
                    metadata.timestamp = HAL_GetTick();
                    strncpy(metadata.raw, metadata_raw, sizeof(metadata.raw) - 1);

                    if (strncmp(&metadata.raw[3], "GGA", 3) == 0) { metadata.valid = interpretGGAStr(metadata_raw); }
                    else { metadata.valid = VALIDITY_VALID_PARTIAL_DATA; }

                    metadata.latitude = metadata_latitude;
                    metadata.longitude = metadata_longitude;
                    metadata.altitude = metadata_altitude;
                    metadata.satellites = metadata_satellites;
                    metadata.hours = metadata_hours;
                    metadata.minutes = metadata_minutes;
                    metadata.seconds = metadata_seconds;
                    #ifdef CM7_SENSOR_GPS_DEBUG
                    char metadata_str[METADATA_GPS_STR_SIZE];
                    sensorGPS_printf("%s", MetadataGPS_toString(&metadata, metadata_str));
                    #endif
                    xQueueOverwrite(metadata_queue, &metadata);
                    
                    metadata_raw_head = 0; // Reset for the next sentence
                }
                else 
                {
                    if (metadata_raw_head < sizeof(metadata_raw)-1) { metadata_raw[metadata_raw_head++] = c; }
                    else
                    {
                        metadata_raw[metadata_raw_head] = '\0'; // Null-terminate the long sentence
                        sensorGPS_printf("GPS sentence too long, discarding: %s", metadata_raw);
                        metadata_raw_head = 0;
                    }
                }
            }
        }
    }
}



static void uartDMAHandler_start(void)
{
    uart_dma_head = 0;
    uart_dma_tail = 0;
    if (HAL_UARTEx_ReceiveToIdle_DMA(&uart_handler, uart_dma_buf, SENSOR_GPS_DMA_BUFFER_SIZE) != HAL_OK) { Error_Handler(); }
}



static Validity_e interpretGGAStr(const char *gpgga)
{
    return VALIDITY_VALID;
}

/////////////////////////////////////////////////////////////////////////////////////////

// Called on DMA half-transfer, transfer-complete, and UART idle line
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart->Instance != UART7) { return; }

    uart_dma_head = size % SENSOR_GPS_DMA_BUFFER_SIZE;
    if (sensorGPS_task != NULL)
    {
        BaseType_t woken = pdFALSE;
        vTaskNotifyGiveFromISR(sensorGPS_task, &woken);
        portYIELD_FROM_ISR(woken);
    }
}

// Overrun, framing, noise, or DMA errors stop the reception, so restart it
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != UART7) { return; }
    HAL_UART_AbortReceive(huart);
    uart_dma_error = 1;
}

void UART7_IRQHandler(void) { HAL_UART_IRQHandler(&uart_handler); }
void DMA1_Stream0_IRQHandler(void) { HAL_DMA_IRQHandler(&uart_dma_handler); }