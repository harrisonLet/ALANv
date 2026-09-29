#include "main.h"
#include "stm32h7xx_hal_i2c.h"
#include <stdint.h>
#include "sensorEncoder.h"
#include <math.h>

#ifndef AS5600_REGS_H
#define AS5600_REGS_H

/* 7-bit I2C address */
#define AS5600_ADDR             0x36

/* Configuration registers */
#define AS5600_ZMCO             0x00
#define AS5600_ZPOS_H           0x01  /* ZPOS[11:8] */
#define AS5600_ZPOS_L           0x02  /* ZPOS[7:0]  */
#define AS5600_MPOS_H           0x03  /* MPOS[11:8] */
#define AS5600_MPOS_L           0x04  /* MPOS[7:0]  */
#define AS5600_MANG_H           0x05  /* MANG[11:8] */
#define AS5600_MANG_L           0x06  /* MANG[7:0]  */
#define AS5600_CONF_H           0x07  /* WD, FTH[2:0], SF[1:0] */
#define AS5600_CONF_L           0x08  /* PWMF, OUTS, HYST, PM  */

/* Output registers */
#define AS5600_RAW_ANGLE_H      0x0C  /* RAW_ANGLE[11:8] */
#define AS5600_RAW_ANGLE_L      0x0D  /* RAW_ANGLE[7:0]  */
#define AS5600_ANGLE_H          0x0E  /* ANGLE[11:8] */
#define AS5600_ANGLE_L          0x0F  /* ANGLE[7:0]  */

/* Status registers */
#define AS5600_STATUS           0x0B
#define AS5600_AGC              0x1A
#define AS5600_MAGNITUDE_H      0x1B  /* MAGNITUDE[11:8] */
#define AS5600_MAGNITUDE_L      0x1C  /* MAGNITUDE[7:0]  */

/* Burn command */
#define AS5600_BURN             0xFF
#define AS5600_BURN_ANGLE           0x80
#define AS5600_BURN_SETTING         0x40

/* STATUS bits */
#define AS5600_STATUS_MH            (1U << 3)  /* magnet too strong */
#define AS5600_STATUS_ML            (1U << 4)  /* magnet too weak   */
#define AS5600_STATUS_MD            (1U << 5)  /* magnet detected   */

/* 12-bit value mask for H/L register pairs */
#define AS5600_12BIT_MASK           0x0FFF

#endif /* AS5600_REGS_H */

// Helper macro to split a float into printable integer parts
#define FLOAT_INT(f)  ((int)(f))
#define FLOAT_FRAC(f) ((int)(fabsf((f) - (int)(f)) * 10000))

static HAL_StatusTypeDef AS5600_readRegs(uint8_t reg, uint8_t *buf, uint16_t len);
static HAL_StatusTypeDef AS5600_writeReg(uint8_t reg, uint8_t value);
static HAL_StatusTypeDef AS5600_read12(uint8_t regH, uint16_t *value);
static void fillStruct();
static void printEncoder();

#pragma pack(1) // ensure no padding between fields
typedef struct {
    uint16_t raw_angle;   // 0-4095, unscaled angle
    uint16_t angle;       // 0-4095, scaled by ZPOS/MPOS/MANG (same as raw_angle until those are programmed)
    uint16_t magnitude;   // internal CORDIC magnitude
    uint8_t  status;      // MD/ML/MH bits
    uint8_t  agc;         // gain: 0-255 at 5V, 0-128 at 3.3V (aim for the middle)

    // Claude said to have ths for when we transfer between cores
    uint32_t update_count;
} Encoder_Data;
#pragma pack()

Encoder_Data encoder = {0};
TaskHandle_t task_sensorEncoder;
I2C_HandleTypeDef I2C_AS5600_Handle;

/**
  * Initialize the hardware.
  */
void sensorEncoder_hardwareInit()
{
    // Using PB6 I2C1_SCL and PB7 I2C1_SDA
    // added __HAL_RCC_I2C1_CLK_ENABLE(); to the main.c
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // using PB7 for I2C1_SDA
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD; // Open Drain - OD
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // using PB6 for I2C1_SCL
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *hi2c);                      line 601 of Stm32h7xx_hal_i2c.h
    // I2C_TypeDef                *Instance;      /*!< I2C registers base address    line 186 of Stm32h7xx_hal_i2c.h
    // I2C_InitTypeDef            Init;           /*!< I2C communication parameters  line 187 of Stm32h7xx_hal_i2c.h
    I2C_AS5600_Handle.Instance = I2C1;

    //<---------------Connor and Charbel Timing Setup----------------->
    // Use table example in reference manual (use 64Mhz base clock divide by 16 to get 4MHz)
    // For each frequency the tables prescales each frequency into 4Mhz and uses 4Mhz for every other settings
    // That's why prescale is 15 since 64MHz / 16 = 4MHz
    //
    // I2C TIMINGR register layout (RM0433 reference manual):
    // [31:28] PRESC  - Prescaler: divides I2C kernel clock. tick = 1/(f_i2cclk / (PRESC+1))
    // [27:24] (reserved, must be 0)
    // [23:20] SCLDEL - SCL data setup delay (in prescaled ticks)
    // [19:16] SDADEL - SDA data hold delay  (in prescaled ticks)
    // [15:8]  SCLH   - SCL high period      (in prescaled ticks, actual = SCLH+1)
    // [7:0]   SCLL   - SCL low period       (in prescaled ticks, actual = SCLL+1)
    //
    // With 64MHz kernel clock and PRESC=15: tick = 1/(64MHz/16) = 250ns
    // SCLDEL=4 -> setup  = 5   * 250ns = 1250ns
    // SDADEL=1 -> hold   = 1   * 250ns =  250ns (AS5600 max data hold is 450ns)
    // SCLH=15  -> high   = 16  * 250ns = 4000ns
    // SCLL=19  -> low    = 20  * 250ns = 5000ns
    // f_SCL = 1 / (4000ns + 5000ns) ~= 111kHz (standard-mode 100kHz, rise/fall times account for the rest)
    I2C_AS5600_Handle.Init.Timing =
        (0xFU << 28) |  // PRESC  = 15 : 64MHz / 16 = 4MHz (250ns per tick)
        (0x0U << 24) |  // reserved
        (0x4U << 20) |  // SCLDEL =  4 : SCL data setup  = 5   ticks = 1.25us
        (0x1U << 16) |  // SDADEL =  1 : SDA data hold   = 1   ticks = 250ns
        (0x0FU << 8) |  // SCLH   = 15 : SCL high period = 16  ticks = 4us
        (0x13U << 0);   // SCLL   = 19 : SCL low  period = 20  ticks = 5us

    I2C_AS5600_Handle.Init.AddressingMode =  I2C_ADDRESSINGMODE_7BIT;
    I2C_AS5600_Handle.Init.OwnAddress1 = 0x1;
    I2C_AS5600_Handle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    I2C_AS5600_Handle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    I2C_AS5600_Handle.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&I2C_AS5600_Handle) != HAL_OK) {
        printf("I2C1 (AS5600) Init Error");
        Error_Handler();
    }

    // AS5600 power-up time T_PU = 10ms max (datasheet p.8)
    HAL_Delay(10);

    // AS5600 has no WHO_AM_I register, so an ACK on its address is the check
    if (HAL_I2C_IsDeviceReady(&I2C_AS5600_Handle, AS5600_ADDR << 1, 3, 100) != HAL_OK) {
        printf("AS5600 not responding at 0x%02X, I2C error: 0x%lX\r\n", AS5600_ADDR, HAL_I2C_GetError(&I2C_AS5600_Handle));
        return;
    }

    uint8_t status = 0;
    if (AS5600_readRegs(AS5600_STATUS, &status, 1) == HAL_OK) {
        printf("AS5600 STATUS 0x%02X: MD=%d ML=%d MH=%d\r\n", status,
               !!(status & AS5600_STATUS_MD), !!(status & AS5600_STATUS_ML), !!(status & AS5600_STATUS_MH));
    }
}

/**
  * Handler for the task.
  */
void sensorEncoder_handler(void *argument)
{
    for(;;)
    {
        fillStruct();
        printEncoder();
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for demonstration purposes
    }
}

/**
  * Read len bytes starting at reg. The address pointer auto-increments, so
  * len > 1 reads consecutive registers.
  */
static HAL_StatusTypeDef AS5600_readRegs(uint8_t reg, uint8_t *buf, uint16_t len)
{
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&I2C_AS5600_Handle, AS5600_ADDR << 1, reg,
                                                I2C_MEMADD_SIZE_8BIT, buf, len, 100);
    if (status != HAL_OK) {
        printf("AS5600 read 0x%02X failed, HAL status: %d, I2C error: 0x%lX\r\n",
               reg, status, HAL_I2C_GetError(&I2C_AS5600_Handle));
    }
    return status;
}

/**
  * Write one byte to reg. Never use this on AS5600_BURN: that permanently
  * programs the one-time memory.
  */
static HAL_StatusTypeDef AS5600_writeReg(uint8_t reg, uint8_t value)
{
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&I2C_AS5600_Handle, AS5600_ADDR << 1, reg,
                                                 I2C_MEMADD_SIZE_8BIT, &value, 1, 100);
    if (status != HAL_OK) {
        printf("AS5600 write 0x%02X failed, HAL status: %d, I2C error: 0x%lX\r\n",
               reg, status, HAL_I2C_GetError(&I2C_AS5600_Handle));
    }
    return status;
}

/**
  * Read a 12-bit H/L register pair. Pass the _H register (e.g. AS5600_RAW_ANGLE_H):
  * ANGLE, RAW ANGLE and MAGNITUDE only handle the address pointer correctly when
  * the read starts at the high byte (datasheet p.13).
  */
static HAL_StatusTypeDef AS5600_read12(uint8_t regH, uint16_t *value)
{
    uint8_t buf[2];
    HAL_StatusTypeDef status = AS5600_readRegs(regH, buf, 2);
    if (status == HAL_OK) {
        *value = ((buf[0] << 8) | buf[1]) & AS5600_12BIT_MASK;
    }
    return status;
}

static void fillStruct()
{
    // Read into locals: the struct is packed, so taking &encoder.field gives an unaligned pointer
    uint16_t raw_angle = 0, angle = 0, magnitude = 0;
    uint8_t status = 0, agc = 0;

    AS5600_read12(AS5600_RAW_ANGLE_H, &raw_angle);
    AS5600_read12(AS5600_ANGLE_H, &angle);
    AS5600_read12(AS5600_MAGNITUDE_H, &magnitude);
    AS5600_readRegs(AS5600_STATUS, &status, 1);
    AS5600_readRegs(AS5600_AGC, &agc, 1);

    encoder.raw_angle = raw_angle;
    encoder.angle     = angle;
    encoder.magnitude = magnitude;
    encoder.status    = status;
    encoder.agc       = agc;
    encoder.update_count++;
}

static void printEncoder()
{
    float degrees = encoder.raw_angle * 360.0f / 4096.0f;

    printf("AS5600 raw: %4u (%d.%04d deg) | STATUS MD=%d ML=%d MH=%d | AGC: %u | MAG: %u\r\n",
           encoder.raw_angle, FLOAT_INT(degrees), FLOAT_FRAC(degrees),
           !!(encoder.status & AS5600_STATUS_MD), !!(encoder.status & AS5600_STATUS_ML),
           !!(encoder.status & AS5600_STATUS_MH), encoder.agc, encoder.magnitude);
}
