/////////////////////////////////////////////////////////////////////////////////////////
// Encoder sensor driver for reading angular position of the main sail relative to the
// vessel.
/////////////////////////////////////////////////////////////////////////////////////////

#include "system.h"

#include "sensorEncoder.h"
#include "objRing.h"

#include "queue.h"

/////////////////////////////////////////////////////////////////////////////////////////

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
#define AS5600_BURN_ANGLE       0x80
#define AS5600_BURN_SETTING     0x40

/* STATUS bits */
#define AS5600_STATUS_MH        (1U << 3)  /* magnet too strong */
#define AS5600_STATUS_ML        (1U << 4)  /* magnet too weak   */
#define AS5600_STATUS_MD        (1U << 5)  /* magnet detected   */

/* 12-bit value mask for H/L register pairs */
#define AS5600_12BIT_MASK           0x0FFF

#endif /* AS5600_REGS_H */

/////////////////////////////////////////////////////////////////////////////////////////

TaskHandle_t sensorEncoder_task;

/////////////////////////////////////////////////////////////////////////////////////////

static I2C_HandleTypeDef i2c_handler;

static QueueHandle_t queue_metadata;
static QueueHandle_t queue_telemetry;

static MetadataEncoder_t storage_metadata[SENSOR_ENCODER_WINDOW];
static Ring_o ring_metadata;
static uint32_t sum_metadata_angle;
static int16_t prev_telemetry_ang_forward;
static uint32_t integral_telemetry_ang;
static uint8_t sequence;

// Must divide 256 so the read cadence survives the uint8_t sequence wrap.
static uint16_t cached_magnitude;
static uint8_t cached_agc;

static HAL_StatusTypeDef AS5600_readRegs(uint8_t reg, uint8_t *buf, uint16_t len);
static HAL_StatusTypeDef AS5600_writeReg(uint8_t reg, uint8_t value);
static HAL_StatusTypeDef AS5600_read(uint8_t seq, MetadataEncoder_t *metadata);

/////////////////////////////////////////////////////////////////////////////////////////

/**
  * Initialize the hardware.
  */
void sensorEncoder_hardwareInit()
{
    sensorEncoder_printf("Initializing Main Sail Encoder hardware and software.");

    // Initialize the Queues used to share data
    sensorEncoder_printf("    - init shared queues");
    queue_metadata =  xQueueCreate(1, sizeof(MetadataEncoder_t));
    queue_telemetry = xQueueCreate(1, sizeof(TelemetryMainSail_t));

    // Initialize the windows used to integrate data
    sensorEncoder_printf("    - init ring buffer for windowed sensor reading");
    objRing_init(&ring_metadata, storage_metadata, sizeof(MetadataEncoder_t), SENSOR_ENCODER_WINDOW);
    sum_metadata_angle = 0;
    prev_telemetry_ang_forward = 0;
    integral_telemetry_ang = 0;

    // Initialize clocks
    sensorEncoder_printf("    - init hardware clocks");
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();
    
    // Initialize GPIOs, Using PB6 I2C1_SCL and PB7 I2C1_SDA
    sensorEncoder_printf("    - init hardware GPIO pins");
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_7 | GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD; // Open Drain - OD
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Initialize I2C handle structure
    // Our system uses I2C with a 64MHz clock, ie f_i2cclk = 64MHz. We scale this down 
    // to 4MHz using a prescaler of 16 (PRESC = 15), 64MHz / 16 = 4MHz. Once there we used
    // the standard 100kHz I2C timing settings described in the reference manual (reference
    // manual RM0399 pg 2138), any of the tables will do since all of the register values
    // except prescaler are the same for all frequencies.
    //
    // I2C TIMINGR register layout (reference manual RM0399 pg 2162)
    sensorEncoder_printf("    - init I2C handle structure");
    i2c_handler.Instance = I2C1;
    i2c_handler.Init.Timing = 0;
    i2c_handler.Init.Timing =
        (15 << 28) |  // PRESC  = 15 : 64MHz / 16 = 4MHz (250ns per tick)
        (4 << 20) |   // SCLDEL =  4 : SCL data setup  = 5   ticks = 1.25us
        (2 << 16) |   // SDADEL =  2 : SDA data hold   = 2   ticks = 0.5us
        (15 << 8) |   // SCLH   = 15 : SCL high period = 16  ticks = 4us
        (19 << 0);    // SCLL   = 19 : SCL low  period = 20  ticks = 5us
    // f_SCL = 1 / (4us + 5us) ~= 111kHz (standard-mode 100kHz, rise/fall times account for the rest)
    i2c_handler.Init.AddressingMode =  I2C_ADDRESSINGMODE_7BIT;
    i2c_handler.Init.OwnAddress1 = 0x1;
    i2c_handler.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    i2c_handler.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    i2c_handler.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&i2c_handler) != HAL_OK) { Error_Handler(); }

    // Initialize AS5600 sensor
    sensorEncoder_printf("    - init AS5600 sensor");
    // AS5600 power-up time T_PU = 10ms max (datasheet p.8)
    HAL_Delay(10);
    // AS5600 has no WHO_AM_I register, so an ACK on its address is the check
    if (HAL_I2C_IsDeviceReady(&i2c_handler, AS5600_ADDR << 1, 3, 100) != HAL_OK) {
        sensorEncoder_printf("ERROR: AS5600 not responding at 0x%02X, I2C error: 0x%lX\r\n", AS5600_ADDR, HAL_I2C_GetError(&i2c_handler));
        return;
    }
    uint8_t status = 0;
    if (AS5600_readRegs(AS5600_STATUS, &status, 1) == HAL_OK) {
        sensorEncoder_printf("AS5600 STATUS 0x%02X: MD=%d ML=%d MH=%d\r\n", status,
               !!(status & AS5600_STATUS_MD), !!(status & AS5600_STATUS_ML), !!(status & AS5600_STATUS_MH));
    }

    sensorEncoder_printf("AS5600 initialization complete");
}

/**
  * Handler for the task.
  */
void sensorEncoder_handler(void *argument)
{
    sensorEncoder_printf("Starting sensor encoder task. Sensor will poll every %u ms", 1000 / SENSOR_ENCODER_FREQUENCY);
    TickType_t lastWake = xTaskGetTickCount();
    for(;;)
    {
        // Read Frequency
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(1000 / SENSOR_ENCODER_FREQUENCY));

        // Metdata
        sensorEncoder_printf("Setting up metadata for encoder reading");
        MetadataEncoder_t metadata;
        metadata.valid = VALIDITY_UNKNOWN;
        metadata.sequence = sequence++;
        metadata.timestamp = HAL_GetTick();

        sensorEncoder_printf("Reading registers from AS5600");
        HAL_StatusTypeDef status_read = AS5600_read(metadata.sequence, &metadata);
        if (status_read != HAL_OK)
        {
            sensorEncoder_printf("ERROR: Failed to read AS5600 registers, HAL status: %d", status_read);
            metadata.valid = VALIDITY_ERROR;
        }
        else
        {
            sensorEncoder_printf("Processing AS5600 readings into metadata");
            if      (metadata.status & AS5600_STATUS_MH) { metadata.valid = VALIDITY_UNTRUSTED_OVER_SATURATION; }
            else if (metadata.status & AS5600_STATUS_ML) { metadata.valid = VALIDITY_UNTRUSTED_UNDER_SATURATION; }
            else { metadata.valid = VALIDITY_VALID; }
            
            if (metadata.valid == VALIDITY_VALID) {
                sum_metadata_angle += metadata.angle;
                MetadataEncoder_t metadata_old;
                if (objRing_push(&ring_metadata, &metadata, &metadata_old)) {
                    sum_metadata_angle -= metadata_old.angle;
                }
            }
            else { }

            if (ring_metadata.count < SENSOR_ENCODER_WINDOW) { metadata.valid = VALIDITY_VALID_WINDOW_FILLING; }
        }
        #ifdef CM7_SENSOR_ENCODER_DEBUG
        char metadata_str[METADATA_ENCODER_STR_SIZE];
        sensorEncoder_printf("%s", MetadataEncoder_toString(&metadata, metadata_str));
        #endif
        xQueueOverwrite(queue_metadata, &metadata);

        // Telemetry
        sensorEncoder_printf("Generating telemetry from metadata");
        if (ring_metadata.count == 0) { continue; }
        TelemetryMainSail_t telemetry;
        int16_t ang_forward = (sum_metadata_angle / ring_metadata.count) - 180;
        telemetry.ang_forward = ang_forward;
        integral_telemetry_ang += ang_forward - prev_telemetry_ang_forward;
        if      (integral_telemetry_ang > 0)  { telemetry.ang_starboard = integral_telemetry_ang; telemetry.ang_port = 0; }
        else if (integral_telemetry_ang < 0)  { telemetry.ang_starboard = 0;                      telemetry.ang_port = -integral_telemetry_ang; }
        else                                  { telemetry.ang_starboard = 0;                      telemetry.ang_port = 0; }
        prev_telemetry_ang_forward = ang_forward;
        xQueueOverwrite(queue_telemetry, &telemetry);
    }
}

/**
  * Read len bytes starting at reg. The address pointer auto-increments, so
  * len > 1 reads consecutive registers.
  */
static HAL_StatusTypeDef AS5600_readRegs(uint8_t reg, uint8_t *buf, uint16_t len)
{
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&i2c_handler, AS5600_ADDR << 1, reg,
                                                I2C_MEMADD_SIZE_8BIT, buf, len, 100);
    if (status != HAL_OK) {
        sensorEncoder_printf("ERROR: AS5600 read 0x%02X failed, HAL status: %d, I2C error: 0x%lX\r\n",
               reg, status, HAL_I2C_GetError(&i2c_handler));
    }
    return status;
}

/**
  * Write one byte to reg. Never use this on AS5600_BURN: that permanently
  * programs the one-time memory.
  */
static HAL_StatusTypeDef AS5600_writeReg(uint8_t reg, uint8_t value)
{
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&i2c_handler, AS5600_ADDR << 1, reg,
                                                 I2C_MEMADD_SIZE_8BIT, &value, 1, 100);
    if (status != HAL_OK) {
        sensorEncoder_printf("ERROR: AS5600 write 0x%02X failed, HAL status: %d, I2C error: 0x%lX\r\n",
               reg, status, HAL_I2C_GetError(&i2c_handler));
    }
    return status;
}

/**
  * Fill the AS5600 fields of metadata. STATUS, RAW_ANGLE and ANGLE come from one burst
  * (0x0B-0x0F, each 12-bit pair read high byte first). AGC and MAGNITUDE (0x1A-0x1C)
  * are re-read only every AS5600_DIAG_PERIOD samples; the cached values are used between.
  */
static HAL_StatusTypeDef AS5600_read(uint8_t seq, MetadataEncoder_t *metadata)
{
    uint8_t buf[5];
    HAL_StatusTypeDef status = AS5600_readRegs(AS5600_STATUS, buf, sizeof(buf));
    if (status != HAL_OK) { return status; }
    metadata->status    = buf[0];
    metadata->raw_angle = ((buf[1] << 8) | buf[2]) & AS5600_12BIT_MASK;
    metadata->angle     = ((buf[3] << 8) | buf[4]) & AS5600_12BIT_MASK;

    if ((seq % 128u) == 0)
    {
        uint8_t diag[3];
        status = AS5600_readRegs(AS5600_AGC, diag, sizeof(diag));
        if (status != HAL_OK) { return status; }
        cached_agc       = diag[0];
        cached_magnitude = ((diag[1] << 8) | diag[2]) & AS5600_12BIT_MASK;
    }
    metadata->agc       = cached_agc;
    metadata->magnitude = cached_magnitude;
    return HAL_OK;
}
