#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>
#include <stdbool.h>

// Default I2C address
#define MPU6050_ADDR         0x68

// MPU6050 Registers
#define MPU6050_REG_PWR_MGMT1    0x6B
#define MPU6050_REG_WHO_AM_I     0x75
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_GYRO_XOUT_H  0x43
#define MPU6050_REG_TEMP_OUT_H   0x41
#define MPU6050_REG_CONFIG       0x1A
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_GYRO_CONFIG  0x1B
#define MPU6050_REG_INT_ENABLE   0x38
#define MPU6050_REG_INT_STATUS   0x3A
#define MPU6050_REG_MOT_THR       0x1F
#define MPU6050_REG_MOT_DUR       0x20
#define MPU6050_REG_MOT_DETECT_CTRL 0x69

typedef enum {
    MPU6050_STATE_IDLE = 0,
    MPU6050_STATE_READING,
    MPU6050_STATE_WRITING
} mpu6050_state_t;

typedef struct {
    mpu6050_state_t state;
    uint8_t reg;
    uint8_t *buffer;
    uint8_t length;
    bool finished;
} mpu6050_transaction_t;

typedef enum {
    MPU6050_ACCEL_RANGE_2G  = 0x00,
    MPU6050_ACCEL_RANGE_4G  = 0x08,
    MPU6050_ACCEL_RANGE_8G  = 0x10,
    MPU6050_ACCEL_RANGE_16G = 0x18
} mpu6050_accel_range_t;

typedef enum {
    MPU6050_GYRO_RANGE_250DPS  = 0x00,
    MPU6050_GYRO_RANGE_500DPS  = 0x08,
    MPU6050_GYRO_RANGE_1000DPS = 0x10,
    MPU6050_GYRO_RANGE_2000DPS = 0x18
} mpu6050_gyro_range_t;

typedef enum {
    MPU6050_FILTER_260HZ = 0x00,
    MPU6050_FILTER_184HZ = 0x01,
    MPU6050_FILTER_94HZ  = 0x02,
    MPU6050_FILTER_44HZ  = 0x03,
    MPU6050_FILTER_21HZ  = 0x04,
    MPU6050_FILTER_10HZ  = 0x05,
    MPU6050_FILTER_5HZ   = 0x06
} mpu6050_filter_t;

typedef enum {
    MPU6050_COUNTER_DECREMENT_RESET = 0x00,
    MPU6050_COUNTER_DECREMENT_1 = 0x01,
    MPU6050_COUNTER_DECREMENT_2 = 0x02,
    MPU6050_COUNTER_DECREMENT_4 = 0x03
} mpu6050_counter_t;

static uint8_t mpu6050_acc[6];
static uint8_t mpu6050_gyro[6];
static uint8_t mpu6050_temp[2];

void mpu6050_init(void);

// Non-blocking read/write functions
bool mpu6050_read(uint8_t reg, uint8_t *buffer, uint8_t length);
bool mpu6050_write(uint8_t reg, uint8_t value);

// High-level helper functions
bool mpu6050_readAccel();
bool mpu6050_readGyro();
bool mpu6050_readTemp();
bool mpu6050_whoAmI(uint8_t *id);

bool mpu6050_setAccelRange(mpu6050_accel_range_t range);
bool mpu6050_setGyroRange(mpu6050_gyro_range_t range);
bool mpu6050_setFilter(mpu6050_filter_t filter);

uint16_t mpu6050_getAccelScaleFactor(void);
uint16_t mpu6050_getGyroScaleFactor(void);

uint8_t *mpu6050_getAccelData(void);
uint8_t *mpu6050_getGyroData(void);
uint8_t *mpu6050_getTempData(void);

bool mpu6050_enableMotionInterrupt(void);
bool mpu6050_disableMotionInterrupt(void);
bool mpu6050_setMotionDetectionThreshold(uint8_t threshold);
bool mpu6050_setMotionDetectionDuration(uint8_t duration);
bool mpu6050_setMotionDetectionControl(mpu6050_counter_t control);

bool mpu6050_readInterruptStatus(void);
uint8_t mpu6050_getInterruptStatus(void);

bool mpu6050_setAccelOnDelay(uint8_t delay);

// Call this inside main loop
bool mpu6050_isBusy(void);

#endif