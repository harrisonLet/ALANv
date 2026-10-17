/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

#include "sensorGPS.h"

#include "main.h"
#include "projdefs.h"
#include "queue.h"
#include "sharedSensor.h"

#include <ctype.h>
#include <errno.h>

/////////////////////////////////////////////////////////////////////////////////////////

#define SENSOR_GPS_DMA_BUFFER_SIZE 256u
#define SENSOR_GPS_IRQ_PRIORITY 6u // must be >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY

typedef enum {
    GGA_ID,
    GGA_TIME,
    GGA_LATITUDE,
    GGA_LATITUDE_DIR,
    GGA_LONGITUDE,
    GGA_LONGITUDE_DIR,
    GGA_FIX_VALID,
    GGA_SATELLITES,
    GGA_HDOP,
    GGA_ALTITUDE,
    GGA_ALTITUDE_UNITS,
    GGA_GEOID_HEIGHT,
    GGA_GEOID_HEIGHT_UNITS,
    GGA_DGPS_UPDATE,
    GGA_DGPS_ID
} GGAField_e;

typedef enum {
    PARSE_UNKNOWN,
    PARSE_SKIP,
    PARSE_SUCCESS,
    PARSE_ERROR_NULL,
    PARSE_ERROR_INVALID,
    PARSE_ERROR_RANGE,
    PARSE_ERROR
} ParseResult_e;

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
static volatile uint32_t uart_dma_error; // HAL error code set by the ISR, non-zero means reception must be restarted

static QueueHandle_t metadata_queue;

static char metadata_raw[METADATA_GPS_RAW_STR_SIZE];
static uint8_t metadata_raw_head;
static double metadata_latitude;
static double metadata_longitude;
static float metadata_altitude;
static uint8_t metadata_satellites;
static uint8_t metadata_fix_valid;
static uint8_t metadata_hours;
static uint8_t metadata_minutes;
static uint8_t metadata_seconds;
static uint8_t metadata_sequence;

static HAL_StatusTypeDef uartDMAHandler_start(void);

static Validity_e    parseGGA(const char *gga);
static ParseResult_e parseGGA_TIME(const char *str, uint8_t *hours, uint8_t *minutes, uint8_t *seconds);
static ParseResult_e parseGGA_LATITUDE(const char *str, double *latitude);
static ParseResult_e parseGGA_LATITUDE_DIR(const char *str, double *latitude);
static ParseResult_e parseGGA_LONGITUDE(const char *str, double *longitude);
static ParseResult_e parseGGA_LONGITUDE_DIR(const char *str, double *longitude);
static ParseResult_e parseGGA_FIX_VALID(const char *str, uint8_t *fix_valid);
static ParseResult_e parseGGA_SATELLITES(const char *str, uint8_t *satellites);
static ParseResult_e parseGGA_ALTITUDE(const char *str, float *altitude);

static ParseResult_e parseUint8(const char *str, uint8_t *value);
static ParseResult_e parseUint8Hex(const char *str, uint8_t *value);
static ParseResult_e parseDouble(const char *str, double *value);
static ParseResult_e parseFloat(const char *str, float *value);


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

    sensorGPS_printf("    - init metadata cache");
    metadata_raw_head = 0;
    metadata_latitude = 0;
    metadata_longitude = 0;
    metadata_altitude = 0;
    metadata_satellites = 0;
    metadata_fix_valid = 0;
    metadata_hours = 0;
    metadata_minutes = 0;
    metadata_seconds = 0;
    metadata_sequence = 0;


    sensorGPS_printf("sensorGPS_rtosInit complete");
    return RTOS_INIT_SUCCESS;
}



void sensorGPS_handler(void *argument)
{
    sensorGPS_printf("sensorGPS_handler start.");
    vTaskDelay(5000); // 5 seconds delay for the GPS module to stabilize
    // The loop below restarts the reception if this fails, so a failure here must not hang the task
    HAL_StatusTypeDef start_status = uartDMAHandler_start(); // start UART DMA reception after GPS module stabilizes
    if (start_status != HAL_OK)
    {
        sensorGPS_printf("UART DMA start failed (HAL status %d), will retry", (int)start_status);
        uart_dma_error = HAL_UART_ERROR_DMA;
    }
    else { sensorGPS_printf("UART DMA reception started"); }
    for (;;)
    {
        // Error handling for UART DMA. The HAL has already stopped the DMA before the error callback runs.
        if (uart_dma_error)
        {
            sensorGPS_printf("UART DMA error detected: 0x%lx", (unsigned long)uart_dma_error);
            uart_dma_error = 0; // Clear first so an error during the restart is not lost
            HAL_UART_AbortReceive(&uart_handler);
            HAL_StatusTypeDef status = uartDMAHandler_start();
            if (status != HAL_OK)
            {
                sensorGPS_printf("UART DMA restart failed (HAL status %d), retrying", (int)status);
                uart_dma_error = HAL_UART_ERROR_DMA;
                vTaskDelay(100);
                continue;
            }
        }
        
        // Metadata
        if (ulTaskNotifyTake(pdTRUE, SENSOR_GPS_STALE) == 0)
        {
            MetadataGPS_t metadata = {0};
            metadata.valid = VALIDITY_INVALID_GPS_STALE;
            metadata.sequence = metadata_sequence++;
            metadata.timestamp = HAL_GetTick();
            #ifdef CM7_SENSOR_GPS_DEBUG
            char metadata_str[METADATA_GPS_STR_SIZE];
            sensorGPS_printf("%s", MetadataGPS_toString(&metadata, metadata_str));
            #endif
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

                    if (strncmp(&metadata.raw[3], "GGA", 3) == 0) { metadata.valid = parseGGA(metadata_raw); }
                    else { metadata.valid = VALIDITY_VALID_PARTIAL_DATA; }

                    metadata.latitude = metadata_latitude;
                    metadata.longitude = metadata_longitude;
                    metadata.altitude = metadata_altitude;
                    metadata.satellites = metadata_satellites;
                    metadata.fix_valid = metadata_fix_valid;
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



static HAL_StatusTypeDef uartDMAHandler_start(void)
{
    uart_dma_head = 0;
    uart_dma_tail = 0;

    // The UART is enabled before reception starts, so bytes received since then leave a stale
    // overrun/framing flag and data in the receiver. Clear them or an error fires immediately.
    __HAL_UART_CLEAR_FLAG(&uart_handler, UART_CLEAR_OREF | UART_CLEAR_FEF | UART_CLEAR_NEF | UART_CLEAR_PEF | UART_CLEAR_IDLEF);
    __HAL_UART_SEND_REQ(&uart_handler, UART_RXDATA_FLUSH_REQUEST);

    return HAL_UARTEx_ReceiveToIdle_DMA(&uart_handler, uart_dma_buf, SENSOR_GPS_DMA_BUFFER_SIZE);
}



static Validity_e parseGGA(const char *gga)
{
    char token[16] = {0};
    uint8_t token_index = 0;
    uint8_t crc = 0;
    bool calculate = false;
    
    uint8_t hour = 0;
    uint8_t minute = 0;
    uint8_t second = 0;
    double latitude = 0;
    double longitude = 0;
    float altitude = 0;
    uint8_t satellites = 0;
    uint8_t fix_valid = 0;

    GGAField_e field = GGA_ID;
    while (*gga != '\0')
    {
        if      (*gga == '$') { crc = 0; calculate = true; }
        else if (*gga == '*') {          calculate = false; }
        else if (calculate)   { crc ^= *gga; }

        if (*gga == ',' || *gga == '*')
        {
            token[token_index] = '\0'; // Null-terminate the token
            if (*token != '\0')
            {
                ParseResult_e r = PARSE_UNKNOWN;
                switch (field)
                {
                    case GGA_TIME:          r = parseGGA_TIME(token, &hour, &minute, &second); break;
                    case GGA_LATITUDE:      r = parseGGA_LATITUDE(token, &latitude); break;
                    case GGA_LATITUDE_DIR:  r = parseGGA_LATITUDE_DIR(token, &latitude); break;
                    case GGA_LONGITUDE:     r = parseGGA_LONGITUDE(token, &longitude); break;
                    case GGA_LONGITUDE_DIR: r = parseGGA_LONGITUDE_DIR(token, &longitude); break;
                    case GGA_FIX_VALID:     r = parseGGA_FIX_VALID(token, &fix_valid); break;
                    case GGA_SATELLITES:    r = parseGGA_SATELLITES(token, &satellites); break;
                    case GGA_ALTITUDE:      r = parseGGA_ALTITUDE(token, &altitude); break;
                    default: r = PARSE_SKIP; break;
                }
                if (r != PARSE_SUCCESS && r != PARSE_SKIP)
                {
                    return VALIDITY_ERROR_PARSE_FAIL;
                }
            }
            token_index = 0;
            field++;
        }
        else 
        { 
            if (token_index >= sizeof(token) - 1) { return VALIDITY_ERROR_PARSE_FAIL; }
            token[token_index++] = *gga; 
        }
        gga++;
    }

    if (calculate) { return VALIDITY_INVALID_GPS; } // If we reached the end without encountering '*', the sentence is incomplete

    token[token_index] = '\0'; // Null-terminate the token
    uint8_t rx_crc;
    if (strnlen(token, sizeof(token)) != 2) { return VALIDITY_INVALID_GPS_CHECKSUM; }
    if (parseUint8Hex(token, &rx_crc) != PARSE_SUCCESS) { return VALIDITY_INVALID_GPS_CHECKSUM; }

    if (rx_crc != crc)       { return VALIDITY_INVALID_GPS_CHECKSUM; }
    else if (fix_valid == 0) { return VALIDITY_INVALID_GPS_NO_FIX; }
    
    metadata_latitude = latitude;
    metadata_longitude = longitude;
    metadata_altitude = altitude;
    metadata_satellites = satellites;
    metadata_fix_valid = fix_valid;
    metadata_hours = hour;
    metadata_minutes = minute;
    metadata_seconds = second;
    
    return VALIDITY_VALID;
}

static ParseResult_e parseGGA_TIME(const char *str, uint8_t *hour, uint8_t *minute, uint8_t *second)
{
    if (strnlen(str, 7) < 6) { return PARSE_ERROR_INVALID; }   // need at least 6 chars
    char hh[3];
    char mm[3];
    char ss[3];
    hh[0] = str[0]; hh[1] = str[1]; hh[2] = '\0';
    mm[0] = str[2]; mm[1] = str[3]; mm[2] = '\0';
    ss[0] = str[4]; ss[1] = str[5]; ss[2] = '\0';
    ParseResult_e r;
    if ((r = parseUint8(hh, hour)) != PARSE_SUCCESS)   { return r; }
    if ((r = parseUint8(mm, minute)) != PARSE_SUCCESS) { return r; }
    if ((r = parseUint8(ss, second)) != PARSE_SUCCESS) { return r; }
    return PARSE_SUCCESS;
}
static ParseResult_e parseGGA_LATITUDE(const char *str, double *latitude)
{
    double raw;
    if (parseDouble(str, &raw) != PARSE_SUCCESS) { return PARSE_ERROR_INVALID; }
    int degrees = (int)(raw / 100);
    double minutes = raw - (degrees * 100);
    *latitude = degrees + (minutes / 60.0);
    return PARSE_SUCCESS;
}
static ParseResult_e parseGGA_LATITUDE_DIR(const char *str, double *latitude) { *latitude = (str[0] == 'N') ? *latitude : -*latitude; return PARSE_SUCCESS; }
static ParseResult_e parseGGA_LONGITUDE(const char *str, double *longitude)
{
    double raw;
    if (parseDouble(str, &raw) != PARSE_SUCCESS) { return PARSE_ERROR_INVALID; }
    int degrees = (int)(raw / 100);
    double minutes = raw - (degrees * 100);
    *longitude = degrees + (minutes / 60.0);
    return PARSE_SUCCESS;
}
static ParseResult_e parseGGA_LONGITUDE_DIR(const char *str, double *longitude) { *longitude = (str[0] == 'E') ? *longitude : -*longitude; return PARSE_SUCCESS; }
static ParseResult_e parseGGA_FIX_VALID(const char *str, uint8_t *fix_valid) { return parseUint8(str, fix_valid); }
static ParseResult_e parseGGA_SATELLITES(const char *str, uint8_t *satellites) { return parseUint8(str, satellites); }
static ParseResult_e parseGGA_ALTITUDE(const char *str, float *altitude) { return parseFloat(str, altitude); }

static ParseResult_e parseUint8(const char *str, uint8_t *value)
{
    if (str == NULL || value == NULL || *str == '\0') { return PARSE_ERROR_NULL; }

    char *end;
    errno = 0;
    unsigned long parsed = strtoul(str, &end, 10);

    if (end == str || *end != '\0') { return PARSE_ERROR_INVALID; }
    if (errno == ERANGE || parsed > UINT8_MAX) { return PARSE_ERROR_RANGE; }
    if (*str == '-') { return PARSE_ERROR_INVALID; } // strtoul silently wraps negatives

    *value = (uint8_t)parsed;
    return PARSE_SUCCESS;
}
static ParseResult_e parseUint8Hex(const char *str, uint8_t *value)
{
    if (str == NULL || value == NULL || *str == '\0') { return PARSE_ERROR_NULL; }
    if (!isxdigit((unsigned char)str[0])) { return PARSE_ERROR_INVALID; } // rejects "-", "+", spaces and "0x"

    char *end;
    errno = 0;
    unsigned long parsed = strtoul(str, &end, 16);

    if (end == str || *end != '\0') { return PARSE_ERROR_INVALID; }
    if (errno == ERANGE || parsed > UINT8_MAX) { return PARSE_ERROR_RANGE; }
    if (*str == '-') { return PARSE_ERROR_INVALID; } // strtoul silently wraps negatives

    *value = (uint8_t)parsed;
    return PARSE_SUCCESS;
}
static ParseResult_e parseDouble(const char *str, double *value)
{
    if (str == NULL || value == NULL || *str == '\0') { return PARSE_ERROR_NULL; }

    char *end;
    errno = 0;
    double parsed = strtod(str, &end);

    if (end == str || *end != '\0') { return PARSE_ERROR_INVALID; }
    if (errno == ERANGE) { return PARSE_ERROR_RANGE; }

    *value = parsed;
    return PARSE_SUCCESS;
}
static ParseResult_e parseFloat(const char *str, float *value)
{
    if (str == NULL || value == NULL || *str == '\0') { return PARSE_ERROR_NULL; }

    char *end;
    errno = 0;
    float parsed = strtof(str, &end);

    if (end == str || *end != '\0') { return PARSE_ERROR_INVALID; }
    if (errno == ERANGE) { return PARSE_ERROR_RANGE; }

    *value = parsed;
    return PARSE_SUCCESS;
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

// Overrun, framing, noise, or DMA errors stop the reception. The HAL has already aborted the DMA at
// this point, so only record the error and wake the task, which restarts the reception.
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != UART7) { return; }

    uart_dma_error = huart->ErrorCode != HAL_UART_ERROR_NONE ? huart->ErrorCode : HAL_UART_ERROR_DMA;
    if (sensorGPS_task != NULL)
    {
        BaseType_t woken = pdFALSE;
        vTaskNotifyGiveFromISR(sensorGPS_task, &woken);
        portYIELD_FROM_ISR(woken);
    }
}

void UART7_IRQHandler(void) { HAL_UART_IRQHandler(&uart_handler); }
void DMA1_Stream0_IRQHandler(void) { HAL_DMA_IRQHandler(&uart_dma_handler); }

/////////////////////////////////////////////////////////////////////////////////////////

bool sensorGPS_getMetadata(MetadataGPS_t *metadata) { return xQueuePeek(metadata_queue, metadata, 0) == pdTRUE; }