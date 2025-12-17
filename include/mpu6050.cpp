#include "mpu6050.h"
#include "i2c.h"

static mpu6050_transaction_t mpu_tx;
static mpu6050_accel_range_t current_accel_range = MPU6050_ACCEL_RANGE_2G;
static mpu6050_gyro_range_t current_gyro_range = MPU6050_GYRO_RANGE_250DPS;
static mpu6050_filter_t current_filter = MPU6050_FILTER_260HZ;
static uint8_t mpu6050_interrupts = 0;
static uint8_t mpu6050_intStatus = 0;
static uint8_t mpu6050_motControl = 0;

void mpu6050_init(void) {
    mpu_tx.state = MPU6050_STATE_IDLE;
    mpu_tx.finished = true;
    // wake device
    mpu6050_write(MPU6050_REG_PWR_MGMT1, 0x00);
}

bool mpu6050_isBusy(void) {
    return !mpu_tx.finished;
}

bool mpu6050_write(uint8_t reg, uint8_t value) {
    if (!mpu_tx.finished)
        return false; // last transfer still running

    mpu_tx.finished = 0;

    // Start non-blocking write: address, register, &data, len=1, callback.
    i2c_writeAsync(MPU6050_ADDR, reg, &value, 1, [](uint8_t result){
        (void)result;
        mpu_tx.finished = true;
    });

    return true; // write successfully started
}

bool mpu6050_read(uint8_t reg, uint8_t *buffer, uint8_t length) {
    if (!mpu_tx.finished) return false;
    mpu_tx.finished = false;
    i2c_readAsync(MPU6050_ADDR, reg, buffer, length, [](uint8_t result){
        (void)result;
        mpu_tx.finished = true;
    });
    return true;
}

/* HELPER FUNCTIONS */

bool mpu6050_whoAmI(uint8_t *id) {
    return mpu6050_read(MPU6050_REG_WHO_AM_I, id, 1);
}

bool mpu6050_readTemp() {
    if (!mpu6050_read(MPU6050_REG_TEMP_OUT_H, mpu6050_temp, 2))
        return false;

    return true;
}

bool mpu6050_readAccel() {
    if (!mpu6050_read(MPU6050_REG_ACCEL_XOUT_H, mpu6050_acc, 6))
        return false;
    return true;
}

bool mpu6050_readGyro() {
    if (!mpu6050_read(MPU6050_REG_GYRO_XOUT_H, mpu6050_gyro, 6))
        return false;
    return true;
}

bool mpu6050_setAccelRange(mpu6050_accel_range_t range) {
    // Implementation depends on MPU6050 register settings for accel range
    // For example, writing to ACCEL_CONFIG register (0x1C)
    current_accel_range = range;
    return mpu6050_write(MPU6050_REG_ACCEL_CONFIG, range);
}

bool mpu6050_setGyroRange(mpu6050_gyro_range_t range) {
    // Implementation depends on MPU6050 register settings for gyro range
    // For example, writing to GYRO_CONFIG register (0x1B)
    current_gyro_range = range;
    return mpu6050_write(MPU6050_REG_GYRO_CONFIG, range);
}

bool mpu6050_setFilter(mpu6050_filter_t filter) {
    // Implementation depends on MPU6050 register settings for digital low pass filter
    // For example, writing to CONFIG register (0x1A)
    current_filter = filter;
    return mpu6050_write(MPU6050_REG_CONFIG, filter);
}

uint16_t mpu6050_getAccelScaleFactor(void) {
    switch (current_accel_range) {
        case MPU6050_ACCEL_RANGE_2G:
            return 16384; // LSB/g
        case MPU6050_ACCEL_RANGE_4G:
            return 8192;  // LSB/g
        case MPU6050_ACCEL_RANGE_8G:
            return 4096;  // LSB/g
        case MPU6050_ACCEL_RANGE_16G:
            return 2048;  // LSB/g
        default:
            return 16384; // default to 2G
    }
}

uint16_t mpu6050_getGyroScaleFactor(void) {
    switch (current_gyro_range) {
        case MPU6050_GYRO_RANGE_250DPS:
            return 1310; // LSB/°/s
        case MPU6050_GYRO_RANGE_500DPS:
            return 655;  // LSB/°/s
        case MPU6050_GYRO_RANGE_1000DPS:
            return 328;  // LSB/°/s
        case MPU6050_GYRO_RANGE_2000DPS:
            return 164;  // LSB/°/s
        default:
            return 1310; // default to 250DPS
    }
}

uint8_t *mpu6050_getAccelData(void) {
    return mpu6050_acc;
}

uint8_t *mpu6050_getGyroData(void) {
    return mpu6050_gyro;
}

uint8_t *mpu6050_getTempData(void) {
    return mpu6050_temp;
}

bool mpu6050_enableMotionInterrupt(void) {
    mpu6050_interrupts |= 0x40; // Set the motion interrupt bit
    // Enable interrupts by writing to INT_ENABLE register (0x38)
    return mpu6050_write(MPU6050_REG_INT_ENABLE, mpu6050_interrupts);
}

bool mpu6050_disableMotionInterrupt(void) {
    mpu6050_interrupts &= ~0x40; // Clear the motion interrupt bit
    // Disable interrupts by writing to INT_ENABLE register (0x38)
    return mpu6050_write(MPU6050_REG_INT_ENABLE, mpu6050_interrupts);
}

bool mpu6050_setMotionDetectionThreshold(uint8_t threshold) {
    // Set motion detection threshold by writing to MOT_THR register (0x1F)
    return mpu6050_write(MPU6050_REG_MOT_THR, threshold);
}

bool mpu6050_setMotionDetectionDuration(uint8_t duration) {
    // Set motion detection duration by writing to MOT_DUR register (0x20)
    return mpu6050_write(MPU6050_REG_MOT_DUR, duration);
}

bool mpu6050_readInterruptStatus(void) {
    return mpu6050_read(MPU6050_REG_INT_STATUS, &mpu6050_intStatus, 1);
}

uint8_t mpu6050_getInterruptStatus(void) {
    return mpu6050_intStatus;
}

bool mpu6050_setMotionDetectionControl(mpu6050_counter_t control) {
    mpu6050_motControl &= ~0x03; // Clear current control bits
    mpu6050_motControl |= (control & 0x03); // Set new control bits
    // Set motion detection control by writing to MOT_DETECT_CTRL register (0x69)
    return mpu6050_write(MPU6050_REG_MOT_DETECT_CTRL, mpu6050_motControl);
}

bool mpu6050_setAccelOnDelay(uint8_t delay) {
    mpu6050_motControl &= ~0x30; // Clear current delay bits
    mpu6050_motControl |= ((delay << 4) & 0x30); // Set new delay bits
    // Set accel on delay by writing to MOT_DETECT_CTRL register (0x69)
    return mpu6050_write(MPU6050_REG_MOT_DETECT_CTRL, mpu6050_motControl);
}