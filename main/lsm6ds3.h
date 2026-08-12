#ifndef LSM6DS3_H
#define LSM6DS3_H

#include "esp_log.h"
#include "driver/i2c_master.h"

#define I2C_MASTER_TIMEOUT_MS 1000

#define LSM6DS3_TAG "LSM6DS3"

#define LSM6DS3_SENSOR_ADDR 0x6B /*!< Address of the LSM6DS3 sensor */
#define LSM6DS3_WHO_AM_I_REG_ADDR 0x0F
#define LSM6DS3_WHO_AM_I 0x69

/* register addresses*/
/* Device Information & Embedded Functions */
#define LSM6DS3_FUNC_CFG_ACCESS_REG_ADDR 0x01
#define LSM6DS3_SENSOR_SYNC_TIME_FRAME_REG_ADDR 0x04
#define LSM6DS3_FIFO_CTRL1_REG_ADDR 0x06
#define LSM6DS3_FIFO_CTRL2_REG_ADDR 0x07
#define LSM6DS3_FIFO_CTRL3_REG_ADDR 0x08
#define LSM6DS3_FIFO_CTRL4_REG_ADDR 0x09
#define LSM6DS3_FIFO_CTRL5_REG_ADDR 0x0A
#define LSM6DS3_ORIENT_CFG_G_REG_ADDR 0x0B
#define LSM6DS3_INT1_CTRL_REG_ADDR 0x0D
#define LSM6DS3_INT2_CTRL_REG_ADDR 0x0E

/* Primary Control Registers */
#define LSM6DS3_CTRL1_XL_REG_ADDR 0x10
#define LSM6DS3_CTRL2_G_REG_ADDR 0x11
#define LSM6DS3_CTRL3_C_REG_ADDR 0x12
#define LSM6DS3_CTRL4_C_REG_ADDR 0x13
#define LSM6DS3_CTRL5_C_REG_ADDR 0x14
#define LSM6DS3_CTRL6_C_REG_ADDR 0x15
#define LSM6DS3_CTRL7_G_REG_ADDR 0x16
#define LSM6DS3_CTRL8_XL_REG_ADDR 0x17
#define LSM6DS3_CTRL9_XL_REG_ADDR 0x18
#define LSM6DS3_CTRL10_C_REG_ADDR 0x19

/* Interrupts & Status Registers */
#define LSM6DS3_WAKE_UP_SRC_REG_ADDR 0x1B
#define LSM6DS3_TAP_SRC_REG_ADDR 0x1C
#define LSM6DS3_D6D_SRC_REG_ADDR 0x1D
#define LSM6DS3_STATUS_REG_ADDR 0x1E

/* Sensor Data Output Registers */
#define LSM6DS3_OUT_TEMP_L_REG_ADDR 0x20
#define LSM6DS3_OUT_TEMP_H_REG_ADDR 0x21
#define LSM6DS3_OUTX_L_G_REG_ADDR 0x22
#define LSM6DS3_OUTX_H_G_REG_ADDR 0x23
#define LSM6DS3_OUTY_L_G_REG_ADDR 0x24
#define LSM6DS3_OUTY_H_G_REG_ADDR 0x25
#define LSM6DS3_OUTZ_L_G_REG_ADDR 0x26
#define LSM6DS3_OUTZ_H_G_REG_ADDR 0x27
#define LSM6DS3_OUTX_L_XL_REG_ADDR 0x28
#define LSM6DS3_OUTX_H_XL_REG_ADDR 0x29
#define LSM6DS3_OUTY_L_XL_REG_ADDR 0x2A
#define LSM6DS3_OUTY_H_XL_REG_ADDR 0x2B
#define LSM6DS3_OUTZ_L_XL_REG_ADDR 0x2C
#define LSM6DS3_OUTZ_H_XL_REG_ADDR 0x2D

/* FIFO Status & Data Output Registers */
#define LSM6DS3_FIFO_STATUS1_REG_ADDR 0x3A
#define LSM6DS3_FIFO_STATUS2_REG_ADDR 0x3B
#define LSM6DS3_FIFO_STATUS3_REG_ADDR 0x3C
#define LSM6DS3_FIFO_STATUS4_REG_ADDR 0x3D
#define LSM6DS3_FIFO_DATA_OUT_L_REG_ADDR 0x3E
#define LSM6DS3_FIFO_DATA_OUT_H_REG_ADDR 0x3F

/* Hardware Configuration & Embedded Functions Configuration */
#define LSM6DS3_TIMESTAMP0_REG_ADDR 0x40
#define LSM6DS3_TIMESTAMP1_REG_ADDR 0x41
#define LSM6DS3_TIMESTAMP2_REG_ADDR 0x42
#define LSM6DS3_STEP_TIMESTAMP_L_REG_ADDR 0x49
#define LSM6DS3_STEP_TIMESTAMP_H_REG_ADDR 0x4A
#define LSM6DS3_STEP_COUNTER_L_REG_ADDR 0x4B
#define LSM6DS3_STEP_COUNTER_H_REG_ADDR 0x4C

/* Advanced Interrupt Configuration Registers */
#define LSM6DS3_TAP_CFG_REG_ADDR 0x58
#define LSM6DS3_TAP_THS_6D_REG_ADDR 0x59
#define LSM6DS3_INT_DUR2_REG_ADDR 0x5A
#define LSM6DS3_WAKE_UP_THS_REG_ADDR 0x5B
#define LSM6DS3_WAKE_UP_DUR_REG_ADDR 0x5C
#define LSM6DS3_FREE_FALL_REG_ADDR 0x5D
#define LSM6DS3_MD1_CFG_REG_ADDR 0x5E
#define LSM6DS3_MD2_CFG_REG_ADDR 0x5F

/**
 * CTRL1_XL bits
 */
#define CTRL1_XL_ODR_XL3 (7)
#define CTRL1_XL_ODR_XL2 (6)
#define CTRL1_XL_ODR_XL1 (5)
#define CTRL1_XL_ODR_XL0 (4)
#define CTRL1_XL_FS_XL1 (3)
#define CTRL1_XL_FS_XL0 (2)
#define CTRL1_XL_LPF1_BW_SEL (1)
#define CTRL1_XL_BW0_XL (0)

/**
 * CTRL2_G bits
 */
#define CTRL2_G_ODR_G3 (7)
#define CTRL2_G_ODR_G2 (6)
#define CTRL2_G_ODR_G1 (5)
#define CTRL2_G_ODR_G0 (4)
#define CTRL2_G_FS_G1 (3)
#define CTRL2_G_FS_G0 (2)
#define CTRL2_G_FS_125 (1)

/**
 * CTRL3_C bits
 */
#define CTRL3_C_BOOT (7)
#define CTRL3_C_BDU (6)
#define CTRL3_C_H_LACTIVE (5)
#define CTRL3_C_PP_OD (4)
#define CTRL3_C_IF_INC (3)
// #define CTRL3_C_
// #define CTRL3_C_
// #define CTRL3_C_
// #define CTRL3_C_

/**
 * CTRL4_C bits
 */

/**
 * CTRL5_C bits
 */

/**
 * CTRL6_C bits
 */

/**
 * CTRL7_G bits
 */

/**
 * CTRL8_XL bits
 */

/**
 * CTRL9_XL bits
 */

/**
 * CTRL10_C bits
 */

/**
 * STATUS_REG bits
 */
#define STATUS_REG_TDA (2)
#define STATUS_REG_GDA (1)
#define STATUS_REG_XLDA (0)

typedef struct
{
    float temp;
    float gyro[3];
    float accel[3];

} lsm6ds3_data_t;

void lsm6ds3_init_accel(i2c_master_dev_handle_t dev_handle);
void lsm6ds3_init_gyro(i2c_master_dev_handle_t dev_handle);
void lsm6ds3_init_all(i2c_master_dev_handle_t dev_handle);

uint8_t lsm6ds3_check_temp_data_available(i2c_master_dev_handle_t dev_handle);
uint8_t lsm6ds3_check_gyro_data_available(i2c_master_dev_handle_t dev_handle);
uint8_t lsm6ds3_check_accel_data_available(i2c_master_dev_handle_t dev_handle);

void lsm6ds3_read_temperature(i2c_master_dev_handle_t dev_handle, float *temp);
void lsm6ds3_read_gyroscope(i2c_master_dev_handle_t dev_handle, float g[3]);
void lsm6ds3_read_accelerometer(i2c_master_dev_handle_t dev_handle, float a[3]);
void lsm6ds3_read_raw_data(i2c_master_dev_handle_t dev_handle, lsm6ds3_data_t *data);

void read_modify_write(uint8_t bit_start,
                       uint8_t bit_end,
                       uint8_t field_value,
                       uint8_t *reg);
uint8_t read_bit(uint8_t reg, uint8_t bit_pos);
void write_bit(uint8_t *reg, uint8_t bit_pos, uint8_t value);

#endif