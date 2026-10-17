/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

#include "sensorIMU.h"

#include "main.h"
#include "projdefs.h"
#include "queue.h"

/////////////////////////////////////////////////////////////////////////////////////////

#define BNO055_ADDR  0x28  // ADR pin LOW (default Adafruit breakout). Use 0x29 if ADR pin is HIGH.

#define BNO055_REG_CHIP_ID  0x00
#define BNO055_CHIP_ID 0xA0
#define BNO055_REG_ACC_ID   0x01 
#define BNO055_ACC_ID 0xFB
#define BNO055_REG_MAG_ID   0x02  
#define BNO055_MAG_ID 0x32
#define BNO055_REG_GYRO_ID  0x03 
#define BNO055_GYRO_ID 0x0F

#define BNO055_REG_ACCX_LSB 0x08
#define BNO055_REG_ACCX_MSB 0x09
#define BNO055_REG_ACCY_LSB 0x0A
#define BNO055_REG_ACCY_MSB 0x0B
#define BNO055_REG_ACCZ_LSB 0x0C
#define BNO055_REG_ACCZ_MSB 0x0D
#define BNO055_REG_ACC BNO055_REG_ACCX_LSB

#define BNO055_REG_MAGX_LSB 0x0E
#define BNO055_REG_MAGX_MSB 0x0F
#define BNO055_REG_MAGY_LSB 0x10
#define BNO055_REG_MAGY_MSB 0x11
#define BNO055_REG_MAGZ_LSB 0x12
#define BNO055_REG_MAGZ_MSB 0x13
#define BNO055_REG_MAG BNO055_REG_MAGX_LSB

#define BNO055_REG_GYRX_LSB 0x14
#define BNO055_REG_GYRX_MSB 0x15
#define BNO055_REG_GYRY_LSB 0x16
#define BNO055_REG_GYRY_MSB 0x17
#define BNO055_REG_GYRZ_LSB 0x18
#define BNO055_REG_GYRZ_MSB 0x19
#define BNO055_REG_GYRO BNO055_REG_GYRX_LSB

#define BNO055_REG_EUL_HEADING_LSB 0x1A
#define BNO055_REG_EUL_HEADING_MSB 0x1B
#define BNO055_REG_EUL_ROLL_LSB 0x1C
#define BNO055_REG_EUL_ROLL_MSB 0x1D
#define BNO055_REG_EUL_PITCH_LSB 0x1E
#define BNO055_REG_EUL_PITCH_MSB 0x1F
#define BNO055_REG_EUL BNO055_REG_EUL_HEADING_LSB

#define BNO055_REG_QUATERNION_W_LSB 0x20
#define BNO055_REG_QUATERNION_W_MSB 0x21
#define BNO055_REG_QUATERNION_X_LSB 0x22
#define BNO055_REG_QUATERNION_X_MSB 0x23
#define BNO055_REG_QUATERNION_Y_LSB 0x24
#define BNO055_REG_QUATERNION_Y_MSB 0x25
#define BNO055_REG_QUATERNION_Z_LSB 0x26
#define BNO055_REG_QUATERNION_Z_MSB 0x27
#define BNO055_REG_QUATERNION BNO055_REG_QUATERNION_W_LSB

#define BNO055_REG_LINEAR_ACC_X_LSB 0x28
#define BNO055_REG_LINEAR_ACC_X_MSB 0x29
#define BNO055_REG_LINEAR_ACC_Y_LSB 0x2A
#define BNO055_REG_LINEAR_ACC_Y_MSB 0x2B
#define BNO055_REG_LINEAR_ACC_Z_LSB 0x2C
#define BNO055_REG_LINEAR_ACC_Z_MSB 0x2D
#define BNO055_REG_LINEAR_ACC BNO055_REG_LINEAR_ACC_X_LSB

#define BNO055_REG_GRAVITY_X_LSB 0x2E
#define BNO055_REG_GRAVITY_X_MSB 0x2F
#define BNO055_REG_GRAVITY_Y_LSB 0x30
#define BNO055_REG_GRAVITY_Y_MSB 0x31
#define BNO055_REG_GRAVITY_Z_LSB 0x32
#define BNO055_REG_GRAVITY_Z_MSB 0x33
#define BNO055_REG_GRAVITY BNO055_REG_GRAVITY_X_LSB

#define BNO055_REG_TEMP 0x34 // Temperature register

#define BNO055_REG_CALIB_STATE 0x35 // Calibration staus register
#define BNO055_REG_ST_RESULT 0x36 // Self-test register
#define BNO055_REG_SYS_STATUS 0x39 // System status register
#define BNO055_REG_SYS_ERROR 0x3A // System error register
#define BNO055_REG_OPR_MODE 0x3D // Operation mode register

#define BNO055_REG_CALIB_OFFSET 0x55
#define BNO055_REG_CALIB_OFFSET_SIZE 22u
static uint8_t BNO055_CALIB_OFFSETS[BNO055_REG_CALIB_OFFSET_SIZE] = {
    0xDD, 0xFF, 0xCC, 0xFF, 0xDD, 0xFF,   // accel X, Y, Z
    0xE2, 0xFF, 0x01, 0x00, 0xEF, 0xFD,   // mag   X, Y, Z
    0xFE, 0xFF, 0x01, 0x00, 0x00, 0x00,   // gyro  X, Y, Z
    0xE8, 0x03, 0xE5, 0x01                // accel radius, mag radius
};

typedef enum { // The calibration status register is actually split into 4 fields
    BNO055_CALIB_NONE        = 0x00, // 0b00000000
    BNO055_CALIB_MAG_SUCCESS = 0x03, // 0b00000011
    BNO055_CALIB_ACC_SUCCESS = 0x0C, // 0b00001100
    BNO055_CALIB_GYR_SUCCESS = 0x30, // 0b00110000
    BNO055_CALIB_SYS_SUCCESS = 0xC0, // 0b11000000
    BNO055_CALIB_SUCCESS     = 0xFF  // 0b11111111
} BNO055CalibState_e;
typedef enum { // The self-test result register is split into 4 fields
    BNO055_ST_RESULT_ACC_SUCCESS = 0x01, // 0b00000001
    BNO055_ST_RESULT_MAG_SUCCESS = 0x02, // 0b00000010
    BNO055_ST_RESULT_GYR_SUCCESS = 0x04, // 0b00000100
    BNO055_ST_RESULT_SYS_SUCCESS = 0x08, // 0b00001000
    BNO055_ST_RESULT_FUL_SUCCESS = 0x0F  // 0b00001111
} BNO055STResult_e;
typedef enum {
    BNO055_SYS_STATE_IDLE        = 0x00,
    BNO055_SYS_STATE_ERROR       = 0x01,
    BNO055_SYS_STATE_INIT_PERIPH = 0x02,
    BNO055_SYS_STATE_INIT_SYS    = 0x03,
    BNO055_SYS_STATE_ST          = 0x04,
    BNO055_SYS_STATE_RUN_FUSION  = 0x05,
    BNO055_SYS_STATE_RUN         = 0x06,
} BNO055SysStatus_e;
typedef enum {
    BNO055_SYS_ERR_NO_ERROR          = 0x00,
    BNO055_SYS_ERR_PERIPH_INIT       = 0x01,
    BNO055_SYS_ERR_SYS_INIT          = 0x02,
    BNO055_SYS_ERR_SELF_TEST         = 0x03,
    BNO055_SYS_ERR_REG_MAP_VAL       = 0x04,
    BNO055_SYS_ERR_REG_MAP_ADDR      = 0x05,
    BNO055_SYS_ERR_REG_MAP_WRITE     = 0x06,
    BNO055_SYS_ERR_LOW_POWER_MODE    = 0x07,
    BNO055_SYS_ERR_ACCEL_POWER_MODE  = 0x08,
    BNO055_SYS_ERR_FUSION_CONFIG     = 0x09,
    BNO055_SYS_ERR_SENSOR_CONFIG     = 0x0A
} BNO055SysErr_e;
typedef enum {
    BNO055_OPR_MODE_CONFIG       = 0x00,
    BNO055_OPR_MODE_ACCONLY      = 0x01,
    BNO055_OPR_MODE_MAGONLY      = 0x02,
    BNO055_OPR_MODE_GYROONLY     = 0x03,
    BNO055_OPR_MODE_ACCMAG       = 0x04,
    BNO055_OPR_MODE_ACCGYRO      = 0x05,
    BNO055_OPR_MODE_MAGGYRO      = 0x06,
    BNO055_OPR_MODE_AMG          = 0x07,
    BNO055_OPR_MODE_IMUPLUS      = 0x08,
    BNO055_OPR_MODE_COMPASS      = 0x09,
    BNO055_OPR_MODE_M4G          = 0x0A,
    BNO055_OPR_MODE_NDOF_FMC_OFF = 0x0B,
    BNO055_OPR_MODE_NDOF         = 0x0C
} BNO055OprMode_e;

typedef enum {
    COM_RESULT_OK,
    COM_RESULT_ERROR
} ComResult_e;

/////////////////////////////////////////////////////////////////////////////////////////

TaskHandle_t sensorIMU_task;

/////////////////////////////////////////////////////////////////////////////////////////

static I2C_HandleTypeDef i2c_handler;

static QueueHandle_t metadata_queue;

static ComResult_e BNO055_read(char *reg, uint8_t addr, uint8_t *data, uint8_t len);
static ComResult_e BNO055_readChipId(uint8_t id[1]) __attribute__((unused));
static ComResult_e BNO055_readAccId(uint8_t id[1]) __attribute__((unused));
static ComResult_e BNO055_readMagId(uint8_t id[1]) __attribute__((unused));
static ComResult_e BNO055_readGyroId(uint8_t id[1]) __attribute__((unused));
static ComResult_e BNO055_readAccXYZ(uint16_t accXYZ[3]) __attribute__((unused));
static ComResult_e BNO055_readMagXYZ(int16_t magXYZ[3]) __attribute__((unused));
static ComResult_e BNO055_readGyroXYZ(int16_t gyroXYZ[3]) __attribute__((unused));
static ComResult_e BNO055_readEulerHRP(uint16_t eulerHRP[3]) __attribute__((unused));
static ComResult_e BNO055_readQuatWXYZ(uint16_t quatWXYZ[4]) __attribute__((unused));
static ComResult_e BNO055_readLinAccXYZ(int16_t linAccXYZ[3]) __attribute__((unused));
static ComResult_e BNO055_readGravXYZ(int16_t gravXYZ[3]) __attribute__((unused));
static ComResult_e BNO055_readTemp(uint8_t temp[1]) __attribute__((unused));
static ComResult_e BNO055_readCalibState(BNO055CalibState_e *state) __attribute__((unused));
static ComResult_e BNO055_readSTResult(BNO055STResult_e *result) __attribute__((unused));
static ComResult_e BNO055_readSysStatus(BNO055SysStatus_e *status) __attribute__((unused));
static ComResult_e BNO055_readSysErr(BNO055SysErr_e *err) __attribute__((unused));
static ComResult_e BNO055_readOprMode(BNO055OprMode_e *mode) __attribute__((unused));

static ComResult_e BNO055_write(char *reg, uint8_t addr, uint8_t *data, uint8_t len);
static ComResult_e BNO055_writeOprMode(BNO055OprMode_e mode);
static ComResult_e BNO055_writeCalibOffset(uint8_t offsets[BNO055_REG_CALIB_OFFSET_SIZE]);

/////////////////////////////////////////////////////////////////////////////////////////

HardwareInit_e sensorIMU_hardwareInit(void)
{
    sensorIMU_printf("sensorIMU_hardwareInit start");

    // Initialize clocks
    sensorIMU_printf("    - init hardware clocks");
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C2_CLK_ENABLE();
    
    // Initialize GPIOs, Using PB11 I2C2_SDA and PB10 I2C2_SCL
    sensorIMU_printf("    - init hardware GPIO pins");
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD; // Open Drain - OD
    GPIO_InitStruct.Pull = GPIO_NOPULL; 
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Initialize I2C handle structure
    // Our system uses I2C with a 64MHz clock, ie f_i2cclk = 64MHz. We scale this down 
    // to 4MHz using a prescaler of 16 (PRESC = 15), 64MHz / 16 = 4MHz. Once there we used
    // the standard 100kHz I2C timing settings described in the reference manual (reference
    // manual RM0399 pg 2138), any of the tables will do since all of the register values
    // except prescaler are the same for all frequencies.
    //
    // I2C TIMINGR register layout (reference manual RM0399 pg 2162)
    sensorIMU_printf("    - init I2C handle structure");
    i2c_handler.Instance = I2C2;
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
    
    // BNO055 initialization
    sensorIMU_printf("    - init BNO055 sensor");
    HAL_Delay(pdMS_TO_TICKS(700)); // The BNO055 requires a power-up delay of 650ms
    uint8_t chip_id;
    if (BNO055_readChipId(&chip_id) != COM_RESULT_OK || chip_id != BNO055_CHIP_ID) {
        sensorIMU_printf("ERROR: BNO055 not detected or wrong chip ID: 0x%02X", chip_id);
        return HARDWARE_INIT_FAIL;
    }
    BNO055STResult_e st;
    if (BNO055_readSTResult(&st) != COM_RESULT_OK || st != BNO055_ST_RESULT_FUL_SUCCESS) {
        sensorIMU_printf("ERROR: Failed to read BNO055 self-test result: 0x%02X", st);
        return HARDWARE_INIT_FAIL;
    }

    // Check BNO055 calibration state
    sensorIMU_printf("    - check BNO055 calibration state");
    ComResult_e result;
    uint16_t calibration_attempts;
    uint16_t calibration_hz;
    #ifdef CM7_SENSOR_IMU_PRE_CALIBRATED
    // The following steps can be found on pg 53 of the BNO055 datasheet
    if ((result = BNO055_writeOprMode(BNO055_OPR_MODE_CONFIG)) != COM_RESULT_OK) {
        sensorIMU_printf("ERROR: Failed to set BNO055 to CONFIG mode");
        return HARDWARE_INIT_FAIL;
    }
    HAL_Delay(pdMS_TO_TICKS(25)); // Delay for the BNO055 to switch to CONFIG mode
    if ((result = BNO055_writeCalibOffset(BNO055_CALIB_OFFSETS)) != COM_RESULT_OK) {
        sensorIMU_printf("ERROR: Failed to write BNO055 calibration offsets");
        return HARDWARE_INIT_FAIL;
    }
    calibration_attempts = 100;
    calibration_hz = 10;
    // Calibration can take up to 10 seconds
    #else
    calibration_attempts = 600;
    calibration_hz = 1;
    // Calibration can take up to 6 minutes
    #endif
    if ((result = BNO055_writeOprMode(BNO055_OPR_MODE_NDOF)) != COM_RESULT_OK) {
        sensorIMU_printf("ERROR: Failed to set BNO055 to NDOF mode");
        return HARDWARE_INIT_FAIL;
    }
    HAL_Delay(pdMS_TO_TICKS(25)); // Delay for the BNO055 to switch to NDOF mode
    BNO055CalibState_e calib_state = BNO055_CALIB_NONE;
    uint8_t count = 0;
    while (count < calibration_attempts) {
        if ((result = BNO055_readCalibState(&calib_state)) != COM_RESULT_OK) {
            sensorIMU_printf("ERROR: Failed to read BNO055 calibration state");
            return HARDWARE_INIT_FAIL;
        }
        if (calib_state == BNO055_CALIB_SUCCESS) { sensorIMU_printf("        - BNO055 calibration successful (attempt %03d)", count); break; }
        else { sensorIMU_printf("        - BNO055 calibration in progress (attempt %03d): %02X", count, calib_state); }
        count++;
        HAL_Delay(pdMS_TO_TICKS(1000 / calibration_hz)); // Delay before next calibration state read
    }

    sensorIMU_printf("sensorIMU_hardwareInit complete");
    return HARDWARE_INIT_SUCCESS;
}



RtosInit_e sensorIMU_rtosInit(void)
{
    sensorIMU_printf("sensorIMU_rtosInit start");

    // Initialize the Queues used to share data
    sensorIMU_printf("    - init shared RTOS structures");
    if ((metadata_queue = xQueueCreate(1, sizeof(MetadataEncoder_t))) == NULL)
    {
        sensorIMU_printf("ERROR: Failed to create queue for metadata");
        return RTOS_INIT_FAIL;
    }

    sensorIMU_printf("sensorIMU_rtosInit complete");
    return RTOS_INIT_SUCCESS;
}



void sensorIMU_handler(void *argument)
{

}

/////////////////////////////////////////////////////////////////////////////////////////

bool sensorIMU_getMetadata(MetadataIMU_t *metadata) { return xQueueReceive(metadata_queue, metadata, 0) == pdTRUE; }

/////////////////////////////////////////////////////////////////////////////////////////

static ComResult_e BNO055_read(uint8_t addr, uint8_t *buf, uint8_t len)
{
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&i2c_handler, BNO055_ADDR << 1, addr, I2C_MEMADD_SIZE_8BIT, buf, len, len*2);
    
    if (status != HAL_OK) {
        sensorIMU_printf("ERROR: BNO055 read 0x%02X failed, HAL status: %d, I2C error: 0x%lX\r\n",
               addr, status, HAL_I2C_GetError(&i2c_handler));
        return COM_RESULT_FAIL;
    }

    return COM_RESULT_OK;
}

static ComResult_e BNO055_readChipId(uint8_t id[1]) { return BNO055_read(BNO055_REG_CHIP_ID, id, 1); }
static ComResult_e BNO055_readAccId(uint8_t id[1])  { return BNO055_read(BNO055_REG_ACC_ID, id, 1); }
static ComResult_e BNO055_readMagId(uint8_t id[1])  { return BNO055_read(BNO055_REG_MAG_ID, id, 1); }
static ComResult_e BNO055_readGyroId(uint8_t id[1]) { return BNO055_read(BNO055_REG_GYRO_ID, id, 1); }

static ComResult_e BNO055_readAccXYZ(uint16_t accXYZ[3]) 
{
    uint8_t buf[6];
    ComResult_e result = BNO055_read(BNO055_REG_ACC, buf, 6);
    if (result != COM_RESULT_OK) {
        return result;
    }
    accXYZ[0] = (uint16_t)(buf[1] << 8 | buf[0]);
    accXYZ[1] = (uint16_t)(buf[3] << 8 | buf[2]);
    accXYZ[2] = (uint16_t)(buf[5] << 8 | buf[4]);
    return COM_RESULT_OK;
}
static ComResult_e BNO055_readMagXYZ(int16_t magXYZ[3])
{
    uint8_t buf[6];
    ComResult_e result = BNO055_read(BNO055_REG_MAG, buf, 6);
    if (result != COM_RESULT_OK) {
        return result;
    }
    magXYZ[0] = (int16_t)(buf[1] << 8 | buf[0]);
    magXYZ[1] = (int16_t)(buf[3] << 8 | buf[2]);
    magXYZ[2] = (int16_t)(buf[5] << 8 | buf[4]);
    return COM_RESULT_OK;
}
static ComResult_e BNO055_readGyroXYZ(int16_t gyroXYZ[3])
{
    uint8_t buf[6];
    ComResult_e result = BNO055_read(BNO055_REG_GYRO, buf, 6);
    if (result != COM_RESULT_OK) {
        return result;
    }
    gyroXYZ[0] = (int16_t)(buf[1] << 8 | buf[0]);
    gyroXYZ[1] = (int16_t)(buf[3] << 8 | buf[2]);
    gyroXYZ[2] = (int16_t)(buf[5] << 8 | buf[4]);
    return COM_RESULT_OK;
}
static ComResult_e BNO055_readEulerHRP(uint16_t eulerHRP[3])
{
    uint8_t buf[6];
    ComResult_e result = BNO055_read(BNO055_REG_EUL, buf, 6);
    if (result != COM_RESULT_OK) {
        return result;
    }
    eulerHRP[0] = (uint16_t)(buf[1] << 8 | buf[0]);
    eulerHRP[1] = (uint16_t)(buf[3] << 8 | buf[2]);
    eulerHRP[2] = (uint16_t)(buf[5] << 8 | buf[4]);
    return COM_RESULT_OK;
}
static ComResult_e BNO055_readQuatWXYZ(uint16_t quatWXYZ[4])
{
    uint8_t buf[8];
    ComResult_e result = BNO055_read(BNO055_REG_QUATERNION, buf, 8);
    if (result != COM_RESULT_OK) {
        return result;
    }
    quatWXYZ[0] = (uint16_t)(buf[1] << 8 | buf[0]);
    quatWXYZ[1] = (uint16_t)(buf[3] << 8 | buf[2]);
    quatWXYZ[2] = (uint16_t)(buf[5] << 8 | buf[4]);
    quatWXYZ[3] = (uint16_t)(buf[7] << 8 | buf[6]);
    return COM_RESULT_OK;
}
static ComResult_e BNO055_readLinAccXYZ(int16_t linAccXYZ[3])
{
    uint8_t buf[6];
    ComResult_e result = BNO055_read(BNO055_REG_LINEAR_ACC, buf, 6);
    if (result != COM_RESULT_OK) {
        return result;
    }
    linAccXYZ[0] = (int16_t)(buf[1] << 8 | buf[0]);
    linAccXYZ[1] = (int16_t)(buf[3] << 8 | buf[2]);
    linAccXYZ[2] = (int16_t)(buf[5] << 8 | buf[4]);
    return COM_RESULT_OK;
}
static ComResult_e BNO055_readGravXYZ(int16_t gravXYZ[3])
{
    uint8_t buf[6];
    ComResult_e result = BNO055_read(BNO055_REG_GRAVITY, buf, 6);
    if (result != COM_RESULT_OK) {
        return result;
    }
    gravXYZ[0] = (int16_t)(buf[1] << 8 | buf[0]);
    gravXYZ[1] = (int16_t)(buf[3] << 8 | buf[2]);
    gravXYZ[2] = (int16_t)(buf[5] << 8 | buf[4]);
    return COM_RESULT_OK;
}
static ComResult_e BNO055_readTemp(uint8_t temp[1])
{
    uint8_t buf[1];
    ComResult_e result = BNO055_read(BNO055_REG_TEMP, buf, 1);
    if (result != COM_RESULT_OK) {
        return result;
    }
    temp[0] = (int8_t)buf[0];
    return COM_RESULT_OK;
}

static ComResult_e BNO055_readCalibState(BNO055CalibState_e *state) { return BNO055_read(BNO055_REG_CALIB_STATE, (uint8_t *)state, sizeof(BNO055CalibState_e)); }
static ComResult_e BNO055_readSTResult(BNO055STResult_e *result)    { return BNO055_read(BNO055_REG_ST_RESULT, (uint8_t *)result, sizeof(BNO055STResult_e)); }
static ComResult_e BNO055_readSysStatus(BNO055SysStatus_e *status)  { return BNO055_read(BNO055_REG_SYS_STATUS, (uint8_t *)status, sizeof(BNO055SysStatus_e)); }
static ComResult_e BNO055_readSysErr(BNO055SysErr_e *err)           { return BNO055_read(BNO055_REG_SYS_ERROR, (uint8_t *)err, sizeof(BNO055SysErr_e)); }
static ComResult_e BNO055_readOprMode(BNO055OprMode_e *mode)        { return BNO055_read(BNO055_REG_OPR_MODE, (uint8_t *)mode, sizeof(BNO055OprMode_e)); }



static ComResult_e BNO055_write(uint8_t addr, uint8_t *data, uint8_t len)
{
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&i2c_handler, BNO055_ADDR << 1, addr, I2C_MEMADD_SIZE_8BIT, data, len, 3);
    
    if (status != HAL_OK) {
        sensorIMU_printf("ERROR: BNO055 write 0x%02X failed, HAL status: %d, I2C error: 0x%lX\r\n",
               addr, status, HAL_I2C_GetError(&i2c_handler));
        return COM_RESULT_ERROR;
    }
    
    return COM_RESULT_OK;
}

static ComResult_e BNO055_writeOprMode(BNO055OprMode_e mode) { return BNO055_write(BNO055_REG_OPR_MODE, (uint8_t *)&mode, sizeof(BNO055OprMode_e)); }
static ComResult_e BNO055_writeCalibOffset(uint8_t offsets[BNO055_REG_CALIB_OFFSET_SIZE]) { return BNO055_write(BNO055_REG_CALIB_OFFSET, offsets, BNO055_REG_CALIB_OFFSET_SIZE); }
