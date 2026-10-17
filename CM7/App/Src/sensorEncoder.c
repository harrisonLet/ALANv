/////////////////////////////////////////////////////////////////////////////////////////
// Encoder sensor driver for reading angular position of the main sail relative to the
// vessel.
//
// I2C: PB6 (SCL), PB7 (SDA)
//
// Wrap safe angle:
// The AS5600 reports 0-4095 counts per turn and jumps from 4095 back to 0 at the seam, so
// averaging raw reads breaks near it. Instead we track a continuous "unwrapped" angle:
//   1. First valid read: unwrapped = raw - ENCODER_FORWARD (counts relative to forward). It 
//      is stored in an int32 so it can keep growing past +-2048 (4096 counts per turn).
//   2. Every later read: take the difference from the previous raw read, wrap it to
//      +-2048 (the shortest way around) and add it to unwrapped. Crossing the seam
//      becomes a small step instead of a jump.
//   3. A running sum of the last SENSOR_ENCODER_WINDOW unwrapped values (add the new one,
//      subtract the one leaving the ring buffer) gives a plain average with no seam.
//   4. The average is converted to degrees. ang_forward is that wrapped to -180..179,
//      ang_starboard/ang_port keep the unwrapped value so they count past 180.
//
// Example, sail pointing nearly aft (right at the seam):
//   raw reads:   4090  4094      3      8
//   raw step:      --    +4  -4091     +5     (-4091 wraps to +5)
//   unwrapped:   2042  2046   2051   2056
// Averaging the raw reads gives ~2048, which reads as forward: the opposite side.
// Averaging the unwrapped values gives ~2048 counts from forward = 180 deg, which is correct.
/////////////////////////////////////////////////////////////////////////////////////////

#include "sensorEncoder.h"
#include "objRing.h"
#include "telemetry.h"

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

// Continuous sail angle in encoder counts relative to forward (raw 2048), it never jumps at the 0/4095 seam.
#define ENCODER_COUNTS     4096
#define ENCODER_HALF_TURN  2048
#define ENCODER_FORWARD    2048

/////////////////////////////////////////////////////////////////////////////////////////

TaskHandle_t sensorEncoder_task;

/////////////////////////////////////////////////////////////////////////////////////////

static I2C_HandleTypeDef i2c_handler;

static QueueHandle_t metadata_queue;

static int32_t ang_unwrapped;
static int32_t ang_unwrapped_storage[SENSOR_ENCODER_WINDOW];
static Ring_o  ang_unwrapped_ring;
static int32_t ang_unwrapped_sum;

static uint16_t metadata_angle_prev;

static uint8_t metadata_sequence;
static uint16_t metadata_magnitude_cache;
static uint8_t metadata_agc_cache;

static HAL_StatusTypeDef AS5600_readRegs(uint8_t reg, uint8_t *buf, uint16_t len);
static HAL_StatusTypeDef AS5600_writeReg(uint8_t reg, uint8_t value) __attribute__((unused)); // it is possible we will need to write offsets to the sensor in the future
static HAL_StatusTypeDef AS5600_read(uint8_t seq, MetadataEncoder_t *metadata);

/////////////////////////////////////////////////////////////////////////////////////////

HardwareInit_e sensorEncoder_hardwareInit(void)
{
    sensorEncoder_printf("sensorEncoder_hardwareInit start");

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
    if (HAL_I2C_IsDeviceReady(&i2c_handler, AS5600_ADDR << 1, 3, 100) != HAL_OK) {
        sensorEncoder_printf("ERROR: AS5600 not responding at 0x%02X, I2C error: 0x%lX\r\n", AS5600_ADDR, HAL_I2C_GetError(&i2c_handler));
        Error_Handler();
    }
    uint8_t status = 0;
    if (AS5600_readRegs(AS5600_STATUS, &status, 1) == HAL_OK) {
        sensorEncoder_printf("    - AS5600 STATUS 0x%02X: MD=%d ML=%d MH=%d\r\n", status,
               !!(status & AS5600_STATUS_MD), !!(status & AS5600_STATUS_ML), !!(status & AS5600_STATUS_MH));
    } else {
        sensorEncoder_printf("ERROR: Failed to read AS5600 STATUS register\r\n");
        Error_Handler();
    }

    sensorEncoder_printf("sensorEncoder_hardwareInit complete");
    return HARDWARE_INIT_SUCCESS;
}



RtosInit_e sensorEncoder_rtosInit(void)
{
    sensorEncoder_printf("sensorEncoder_rtosInit start");

    // Initialize the Queues used to share data
    sensorEncoder_printf("    - init shared RTOS structures");
    if ((metadata_queue = xQueueCreate(1, sizeof(MetadataEncoder_t))) == NULL)
    {
        sensorEncoder_printf("ERROR: Failed to create queue for metadata");
        return RTOS_INIT_FAIL;
    }

    // Initialize the windows used to integrate data
    sensorEncoder_printf("    - init ring buffer for windowed sensor reading");
    objRing_init(&ang_unwrapped_ring, ang_unwrapped_storage, sizeof(int32_t), SENSOR_ENCODER_WINDOW);
    ang_unwrapped_sum = 0;
    ang_unwrapped = 0;
    metadata_angle_prev = ENCODER_FORWARD;

    sensorEncoder_printf("sensorEncoder_rtosInit complete");
    return RTOS_INIT_SUCCESS;
}



void sensorEncoder_handler(void *argument)
{
    sensorEncoder_printf("sensorEncoder_handler start. Poll: %u ms", 1000 / SENSOR_ENCODER_FREQUENCY);
    TickType_t lastWake = xTaskGetTickCount();
    for(;;)
    {
        // Read Frequency
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(1000 / SENSOR_ENCODER_FREQUENCY));

        // Metdata
        MetadataEncoder_t metadata = {0};
        metadata.valid = VALIDITY_UNKNOWN;
        metadata.sequence = metadata_sequence++;
        metadata.timestamp = HAL_GetTick();

        HAL_StatusTypeDef status_read = AS5600_read(metadata.sequence, &metadata);
        if (status_read != HAL_OK)
        {
            sensorEncoder_printf("ERROR: Failed to read AS5600 registers, HAL status: %d", status_read);
            metadata.valid = VALIDITY_ERROR;
        }
        else
        {
            if      (metadata.status & AS5600_STATUS_MH) { metadata.valid = VALIDITY_UNTRUSTED_OVER_SATURATION; }
            else if (metadata.status & AS5600_STATUS_ML) { metadata.valid = VALIDITY_UNTRUSTED_UNDER_SATURATION; }
            else                                         { metadata.valid = VALIDITY_VALID; }
            
            if (metadata.valid == VALIDITY_VALID) {
                int32_t delta = (int32_t)metadata.angle - metadata_angle_prev;
                if (delta >  ENCODER_HALF_TURN) { delta -= ENCODER_COUNTS; }
                if (delta < -ENCODER_HALF_TURN) { delta += ENCODER_COUNTS; }
                ang_unwrapped += delta;
                metadata_angle_prev = metadata.angle;

                int32_t ang_unwrapped_old;
                ang_unwrapped_sum += ang_unwrapped;
                if (objRing_push(&ang_unwrapped_ring, &ang_unwrapped, &ang_unwrapped_old)) {
                    ang_unwrapped_sum -= ang_unwrapped_old;
                }
            }
            else { }
        }
        #ifdef CM7_SENSOR_ENCODER_DEBUG
        char metadata_str[METADATA_ENCODER_STR_SIZE];
        sensorEncoder_printf("%s", MetadataEncoder_toString(&metadata, metadata_str));
        #endif
        xQueueOverwrite(metadata_queue, &metadata);

        // Telemetry
        if (ang_unwrapped_ring.count == 0) { continue; }
        TelemetryMainSail_t telemetry;
        int32_t degrees = ((ang_unwrapped_sum / (int32_t)ang_unwrapped_ring.count) * 360) / ENCODER_COUNTS;
        telemetry.ang_forward = (int16_t)((((degrees + 180) % 360) + 360) % 360 - 180);
        if      (degrees > 0) { telemetry.ang_starboard = degrees; telemetry.ang_port = 0; }
        else if (degrees < 0) { telemetry.ang_starboard = 0;       telemetry.ang_port = -degrees; }
        else                  { telemetry.ang_starboard = 0;       telemetry.ang_port = 0; }
        telemetry_setMainSail(&telemetry);
    }
}

/**
  * Read len bytes starting at reg. The address pointer auto-increments, so
  * len > 1 reads consecutive registers.
  */
static HAL_StatusTypeDef AS5600_readRegs(uint8_t reg, uint8_t *buf, uint16_t len)
{
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&i2c_handler, AS5600_ADDR << 1, reg,
                                                I2C_MEMADD_SIZE_8BIT, buf, len, 3);
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
                                                 I2C_MEMADD_SIZE_8BIT, &value, 1, 3);
    if (status != HAL_OK) {
        sensorEncoder_printf("ERROR: AS5600 write 0x%02X failed, HAL status: %d, I2C error: 0x%lX\r\n",
               reg, status, HAL_I2C_GetError(&i2c_handler));
    }
    return status;
}

/**
  * Fill the AS5600 fields of metadata. STATUS, RAW_ANGLE and ANGLE come from one burst
  * (0x0B-0x0F, each 12-bit pair read high byte first). AGC and MAGNITUDE (0x1A-0x1C)
  * are re-read only every 128 samples; the cached values are used between.
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
        metadata_agc_cache       = diag[0];
        metadata_magnitude_cache = ((diag[1] << 8) | diag[2]) & AS5600_12BIT_MASK;
    }
    metadata->agc       = metadata_agc_cache;
    metadata->magnitude = metadata_magnitude_cache;
    return HAL_OK;
}

/////////////////////////////////////////////////////////////////////////////////////////

bool sensorEncoder_getMetadata(MetadataEncoder_t *metadata) { return xQueuePeek(metadata_queue, metadata, 0) == pdTRUE; }
