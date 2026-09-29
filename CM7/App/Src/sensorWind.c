#include "cmsis_os2.h"
#include "main.h"
#include "sensorWind.h"
#include "stm32h755xx.h"
#include "stm32h7xx_hal_def.h"
#include "stm32h7xx_hal_gpio_ex.h"
#include "stm32h7xx_hal_uart.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "servoSail.h"

/* ------------------------------------------------------------------ */
/* Config                                                               */
/* ------------------------------------------------------------------ */

#define SENSOR_ADDRESS  0x02  // Default per protocol spec

/* ------------------------------------------------------------------ */
/* Globals                                                              */
/* ------------------------------------------------------------------ */
TaskHandle_t        task_sensorWind;
UART_HandleTypeDef  UART4_Handler = {0};

/* ------------------------------------------------------------------ */
/* Forward declarations                                                 */
/* ------------------------------------------------------------------ */
static void     sensorWind_uart4Init(void);

static uint16_t crc16(const uint8_t *buf, int len);
static void     append_crc(uint8_t *buf, int len);
static bool     check_crc(const uint8_t *buf, int len);
static void     flush_rx(void);
static bool     read_bytes(uint8_t *buf, size_t len, uint32_t timeout_ms);
static float    read_wind_angle_360(uint8_t addr);
static int      read_wind_dir_16(uint8_t addr);
static const char* direction_name(int val);


// Hardware init                                                       
void sensorWind_hardwareInit(void)
{
    sensorWind_uart4Init();
}

static void sensorWind_uart4Init(void)
{
    /* PC10 — UART4 TX */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin       = GPIO_PIN_10;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* PC11 — UART4 RX */
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* Set UART4 kernel clock explicitly */
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_UART4;
    PeriphClkInit.Usart16ClockSelection  = RCC_UART4CLKSOURCE_D2PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        Error_Handler();
    }

    /* UART4 peripheral */
    UART4_Handler.Instance        = UART4;
    UART4_Handler.Init.BaudRate   = 9600;
    UART4_Handler.Init.WordLength = UART_WORDLENGTH_8B;
    UART4_Handler.Init.StopBits   = UART_STOPBITS_1;
    UART4_Handler.Init.Parity     = UART_PARITY_NONE;
    UART4_Handler.Init.Mode       = UART_MODE_TX_RX;

    if (HAL_UART_Init(&UART4_Handler) != HAL_OK)
    {
        Error_Handler();
    }

    uint32_t apb1_clk = HAL_RCC_GetPCLK1Freq();
    uint32_t sys_clk  = HAL_RCC_GetSysClockFreq();
    printf("UART4 init complete on PC10(TX) / PC11(RX)\r\n");
    printf("  SYSCLK:  %lu Hz\r\n", sys_clk);
    printf("  APB1CLK: %lu Hz\r\n", apb1_clk);
    printf("  Expected UART4 baud: 9600\r\n");
}


void sensorWind_handler(void *argument)
{
    vTaskSuspend(NULL);
    for (;;)
    {
        float angle = read_wind_angle_360(SENSOR_ADDRESS);

        if (angle < 0.0f)
        {
            printf("Wind sensor: timeout or CRC error\r\n");
        }
        else
        {
            uint16_t degrees = (uint16_t)angle;
            uint16_t tenths  = (uint16_t)(angle * 10) % 10;  // get decimal digit
            printf("Wind angle: %u.%u deg\r\n", degrees, tenths);
            servoSail_setAngle((degrees - 180) / -2);
        }
    }
}

// DEBUG Version, prints out what is sent and what is approaved
// void sensorWind_handler(void *argument)
// {

//     printf("SYSCLK:  %lu Hz\r\n", HAL_RCC_GetSysClockFreq());
//     printf("APB1CLK: %lu Hz\r\n", HAL_RCC_GetPCLK1Freq());
//     printf("D3PCLK1: %lu Hz\r\n", HAL_RCCEx_GetD3PCLK1Freq());
    
//     // Use the known-good hardcoded frame from the datasheet for addr 0x02
//     // so we can rule out any CRC calculation issues entirely
//     uint8_t cmd[8] = {0x02, 0x03, 0x00, 0x00, 0x00, 0x01, 0x84, 0x39};

//     // Also print what our CRC function computes, so we can compare
//     uint8_t cmd_check[8] = {0x02, 0x03, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00};
//     append_crc(cmd_check, 6);
//     printf("Datasheet CRC:  84 39\r\n");
//     printf("Computed CRC:   %02X %02X\r\n", cmd_check[6], cmd_check[7]);
//     // These two lines must match. If they don't, there's a clock/compile issue.

//     uint8_t resp[16] = {0}; // oversized buffer to catch any extra bytes

//     for (;;)
//     {
//         memset(resp, 0, sizeof(resp));

//         flush_rx();
//         printf("Sending: %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
//                cmd[0], cmd[1], cmd[2], cmd[3],
//                cmd[4], cmd[5], cmd[6], cmd[7]);

//         HAL_UART_Transmit(&UART4_Handler, cmd, 8, 1000);

//         // Read up to 16 bytes with generous timeout to catch whatever comes back
//         HAL_StatusTypeDef status = HAL_UART_Receive(&UART4_Handler, resp, 7, 3000);

//         if (status == HAL_OK)
//         {
//             printf("RX OK:      %02X %02X %02X %02X %02X %02X %02X\r\n",
//                    resp[0], resp[1], resp[2], resp[3],
//                    resp[4], resp[5], resp[6]);

//             // Tell us exactly what's wrong
//             if (resp[0] != 0x02) printf("  >> addr wrong: got %02X, want 02\r\n", resp[0]);
//             if (resp[1] != 0x03) printf("  >> func wrong: got %02X, want 03\r\n", resp[1]);
//             if (resp[2] != 0x02) printf("  >> len wrong:  got %02X, want 02\r\n", resp[2]);

//             // Check CRC regardless of header
//             bool crc_ok = check_crc(resp, 7);
//             printf("  >> CRC: %s\r\n", crc_ok ? "OK" : "FAIL");
//         }
//         else if (status == HAL_TIMEOUT)
//         {
//             printf("RX TIMEOUT — zero bytes received within 3000ms\r\n");
//             printf("  Check: sensor powered? A+/B- wired to DFR0845? 5V on DFR0845?\r\n");
//         }
//         else
//         {
//             printf("RX ERROR: HAL status %d\r\n", (int)status);
//         }

//         printf("---\r\n");
//         vTaskDelay(pdMS_TO_TICKS(2000));
//     }
// }


// CRC16 Modbus — single consistent implementation                      
// Standard Modbus CRC16, low byte first (little-endian)
static uint16_t crc16(const uint8_t *buf, int len)
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

// Appends CRC low byte then high byte to buf[len], buf[len+1]
static void append_crc(uint8_t *buf, int len)
{
    uint16_t crc  = crc16(buf, len);
    buf[len]      = crc & 0xFF;
    buf[len + 1]  = (crc >> 8) & 0xFF;
}

// Validates CRC in last 2 bytes of buf (low byte first)
static bool check_crc(const uint8_t *buf, int len)
{
    uint16_t computed = crc16(buf, len - 2);
    uint16_t received = (uint16_t)(buf[len - 2] | ((uint16_t)buf[len - 1] << 8));
    return computed == received;
}

/* ------------------------------------------------------------------ */
/* Serial helpers                                                       */
/* ------------------------------------------------------------------ */
static void flush_rx(void)
{
    // Abort any ongoing receive and clear the UART RX FIFO/shift register
    HAL_UART_AbortReceive(&UART4_Handler);
    uint8_t dummy;
    while (HAL_UART_Receive(&UART4_Handler, &dummy, 1, 1) == HAL_OK) {}
}

// Read exactly `len` bytes with a timeout. Returns true on success.
static bool read_bytes(uint8_t *buf, size_t len, uint32_t timeout_ms)
{
    // HAL_UART_Receive blocks until all bytes received or timeout
    HAL_StatusTypeDef status = HAL_UART_Receive(&UART4_Handler, buf, (uint16_t)len, timeout_ms);
    return (status == HAL_OK);
}

/* ------------------------------------------------------------------ */
/* Sensor reads                                                         */
/* ------------------------------------------------------------------ */

// Returns wind angle in degrees (e.g. 65.5f), or -1.0f on error
static float read_wind_angle_360(uint8_t addr)
{
    // Build Modbus read command for register 0x0000 (360° angle)
    uint8_t cmd[8] = {addr, 0x03, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00};
    append_crc(cmd, 6);

    flush_rx();

    if (HAL_UART_Transmit(&UART4_Handler, cmd, 8, 1000) != HAL_OK)
    {
        printf("TX error\r\n");
        return -1.0f;
    }

    // Expect 7 bytes: [addr][0x03][0x02][dataH][dataL][crcL][crcH]
    uint8_t resp[7] = {0};
    if (!read_bytes(resp, 7, 1000))
    {
        printf("RX timeout\r\n");
        return -1.0f;
    }

    // Validate header bytes
    if (resp[0] != addr || resp[1] != 0x03 || resp[2] != 0x02)
    {
        printf("Bad header: %02X %02X %02X\r\n", resp[0], resp[1], resp[2]);
        return -1.0f;
    }

    // Validate CRC
    if (!check_crc(resp, 7))
    {
        printf("CRC error on 360 angle\r\n");
        return -1.0f;
    }

    uint16_t raw = (uint16_t)(((uint16_t)resp[3] << 8) | resp[4]);
    return raw / 10.0f;  // e.g. 655 raw → 65.5°
}

// Returns 0-15 for 16 wind directions, or -1 on error
static int read_wind_dir_16(uint8_t addr)
{
    // Build Modbus read command for register 0x0001 (16-direction)
    uint8_t cmd[8] = {addr, 0x03, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00};
    append_crc(cmd, 6);

    flush_rx();

    if (HAL_UART_Transmit(&UART4_Handler, cmd, 8, 1000) != HAL_OK)
        return -1;

    uint8_t resp[7] = {0};
    if (!read_bytes(resp, 7, 1000))
        return -1;

    if (resp[0] != addr || resp[1] != 0x03 || resp[2] != 0x02)
        return -1;

    if (!check_crc(resp, 7))
        return -1;

    return (int)(((uint16_t)resp[3] << 8) | resp[4]);
}

/* ------------------------------------------------------------------ */
/* Direction name lookup                                                */
/* ------------------------------------------------------------------ */
static const char* direction_name(int val)
{
    static const char *dirs[] = {
        "North",          "North-northeast", "Northeast",  "East-northeast",
        "East",           "East-southeast",  "Southeast",  "South-southeast",
        "South",          "South-southwest", "Southwest",  "West-southwest",
        "West",           "West-northwest",  "Northwest",  "North-northwest"
    };
    if (val < 0 || val > 15) return "Unknown";
    return dirs[val];
}
