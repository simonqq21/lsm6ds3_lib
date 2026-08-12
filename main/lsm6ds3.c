#include "lsm6ds3.h"

static uint8_t data[6];

/**
 * @brief Read a sequence of bytes from a LSM6DS3 sensor registers
 */
static esp_err_t lsm6ds3_register_read(i2c_master_dev_handle_t dev_handle,
                                       uint8_t reg_addr,
                                       uint8_t *data,
                                       size_t len)
{
    /* transmit one byte (the register address on the LSM6DS3) and receive multiple bytes back from the LSM6DS3. */
    return i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS);
}

/**
 * @brief Write a byte to a LSM6DS3 sensor register
 */
static esp_err_t lsm6ds3_register_write_byte(i2c_master_dev_handle_t dev_handle,
                                             uint8_t reg_addr,
                                             uint8_t data)
{
    /* write the register address first, then the data to write to the LSM6DS3. */
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS);
}

/**
 * @brief initialize LSM6DS3 accelerometer
 */
void lsm6ds3_init_accel(i2c_master_dev_handle_t dev_handle)
{
    /* set accelerometer power to normal power */
    // uint8_t test_reg = 0xA5;
    // read_modify_write(4, 0, 16, &test_reg);
    // ESP_LOGI(LSM6DS3_TAG, "test_reg = %X", test_reg);

    ESP_ERROR_CHECK(lsm6ds3_register_read(dev_handle, LSM6DS3_CTRL1_XL_REG_ADDR, data, 1));
    ESP_LOGI(LSM6DS3_TAG, "CTRL1_XL = %X", data[0]);
    /* set accelerometer FS
    FS_XL = 0b10 for +-4G FS */
    read_modify_write(CTRL1_XL_FS_XL1, CTRL1_XL_FS_XL0, 0b10, &data[0]);

    /* set accelerometer ODR
    ODR_XL = 0b0100 for 104 Hz ODR */
    read_modify_write(CTRL1_XL_ODR_XL3, CTRL1_XL_ODR_XL0, 0b0100, &data[0]);

    /* LPF1_BW_SEL and BW0_XL are 0 */

    /* write LSM6DS3_CTRL1_XL_REG_ADDR
    0b01001000 = 0x48*/
    ESP_LOGI(LSM6DS3_TAG, "CTRL1_XL = %X", data[0]);
    ESP_ERROR_CHECK(lsm6ds3_register_write_byte(dev_handle, LSM6DS3_CTRL1_XL_REG_ADDR, data[0]));
    ESP_ERROR_CHECK(lsm6ds3_register_read(dev_handle, LSM6DS3_CTRL1_XL_REG_ADDR, data, 1));
    ESP_LOGI(LSM6DS3_TAG, "CTRL1_XL = %X", data[0]);
}

/**
 * @brief initialize LSM6DS3 gyroscope
 */
void lsm6ds3_init_gyro(i2c_master_dev_handle_t dev_handle)
{
    ESP_ERROR_CHECK(lsm6ds3_register_read(dev_handle, LSM6DS3_CTRL2_G_REG_ADDR, data, 1));
    ESP_LOGI(LSM6DS3_TAG, "CTRL1_G = %X", data[0]);

    /* set gyroscope power to normal power */
    /* set gyroscope FS
    FS_G = 0b01 for +-500 dps */
    read_modify_write(CTRL2_G_FS_G1, CTRL2_G_FS_G0, 0b01, &data[0]);

    /* set gyroscope ODR
    ODR_G = 0b0100 for 104 Hz ODR*/
    read_modify_write(CTRL2_G_ODR_G3, CTRL2_G_ODR_G0, 0b0100, &data[0]);

    /* FS_125 is 0 */
    /* write LSM6DS3_CTRL2_G_REG_ADDR
    0b01000100 = 0x44 */
    ESP_LOGI(LSM6DS3_TAG, "CTRL2_G = %X", data[0]);
    ESP_ERROR_CHECK(lsm6ds3_register_write_byte(dev_handle, LSM6DS3_CTRL2_G_REG_ADDR, data[0]));
    ESP_ERROR_CHECK(lsm6ds3_register_read(dev_handle, LSM6DS3_CTRL2_G_REG_ADDR, data, 1));
    ESP_LOGI(LSM6DS3_TAG, "CTRL2_G = %X", data[0]);
}

/**
 * @brief initialize LSM6DS3
 */
void lsm6ds3_init_all(i2c_master_dev_handle_t dev_handle)
{
    /* read the WHO_AM_I register fromthe LSM6DS3
    WHO_AM_I must be 0x69 */
    ESP_ERROR_CHECK(lsm6ds3_register_read(dev_handle, LSM6DS3_WHO_AM_I_REG_ADDR, data, 1));
    ESP_LOGI(LSM6DS3_TAG, "WHO_AM_I = %X", data[0]);

    /* initialize accelerometer */
    lsm6ds3_init_accel(dev_handle);
    /* initialize gyroscope */
    lsm6ds3_init_gyro(dev_handle);

    /* enable block data update (BDU) */
    ESP_ERROR_CHECK(lsm6ds3_register_read(dev_handle, LSM6DS3_CTRL3_C_REG_ADDR, data, 1));
    read_modify_write(CTRL3_C_BDU, CTRL3_C_BDU, 1, data);
    ESP_ERROR_CHECK(lsm6ds3_register_write_byte(dev_handle, LSM6DS3_CTRL3_C_REG_ADDR, data[0]));
}

/**
 * @brief check if new temperature data is available from the LSM6DS3
 */
uint8_t lsm6ds3_check_temp_data_available(i2c_master_dev_handle_t dev_handle)
{
    ESP_ERROR_CHECK(lsm6ds3_register_read(dev_handle, LSM6DS3_STATUS_REG_ADDR, data, 1));
    return read_bit(data[0], STATUS_REG_TDA);
}

/**
 * @brief check if new gyroscope data is available from the LSM6DS3
 */
uint8_t lsm6ds3_check_gyro_data_available(i2c_master_dev_handle_t dev_handle)
{
    ESP_ERROR_CHECK(lsm6ds3_register_read(dev_handle, LSM6DS3_STATUS_REG_ADDR, data, 1));
    return read_bit(data[0], STATUS_REG_GDA);
}

/**
 * @brief check if new accelerometer data is available from the LSM6DS3
 */
uint8_t lsm6ds3_check_accel_data_available(i2c_master_dev_handle_t dev_handle)
{
    ESP_ERROR_CHECK(lsm6ds3_register_read(dev_handle, LSM6DS3_STATUS_REG_ADDR, data, 1));
    return read_bit(data[0], STATUS_REG_XLDA);
}

/**
 * @brief read temperature from LSM6DS3
 */
void lsm6ds3_read_temperature(i2c_master_dev_handle_t dev_handle, float *temp)
{
    int16_t temp_raw;
    float temp1;
    ESP_ERROR_CHECK(lsm6ds3_register_read(dev_handle, LSM6DS3_OUT_TEMP_L_REG_ADDR, data, 2));
    temp_raw = (data[1] << 8) | data[0];
    ESP_LOGI(LSM6DS3_TAG, "temp_raw = %d", temp_raw);
    temp1 = temp_raw / 256.0 + 25.0;
    ESP_LOGI(LSM6DS3_TAG, "temp1 = %f", temp1);
    *temp = temp_raw / 256.0 + 25.0;
}

/**
 * @brief read gyroscope from LSM6DS3
 */
void lsm6ds3_read_gyroscope(i2c_master_dev_handle_t dev_handle, float g[3])
{
    int16_t g_raw[3];
    g_raw[0] = 0;
    g_raw[1] = 0;
    g_raw[2] = 0;
    ESP_ERROR_CHECK(lsm6ds3_register_read(dev_handle, LSM6DS3_OUTX_L_G_REG_ADDR, data, 6));
    g_raw[0] = (data[1] << 8) | data[0];
    g_raw[1] = (data[3] << 8) | data[2];
    g_raw[2] = (data[5] << 8) | data[4];
    g[0] = g_raw[0] / 32768.0 * 500.0;
    g[1] = g_raw[1] / 32768.0 * 500.0;
    g[2] = g_raw[2] / 32768.0 * 500.0;
}

/**
 * @brief read gyroscope from LSM6DS3
 */
void lsm6ds3_read_accelerometer(i2c_master_dev_handle_t dev_handle, float a[3])
{
    int16_t a_raw[3];
    a_raw[0] = 0;
    a_raw[1] = 0;
    a_raw[2] = 0;
    ESP_ERROR_CHECK(lsm6ds3_register_read(dev_handle, LSM6DS3_OUTX_L_XL_REG_ADDR, data, 6));
    a_raw[0] = (data[1] << 8) | data[0];
    a_raw[1] = (data[3] << 8) | data[2];
    a_raw[2] = (data[5] << 8) | data[4];
    a[0] = a_raw[0] / 32768.0 * 4.0;
    a[1] = a_raw[1] / 32768.0 * 4.0;
    a[2] = a_raw[2] / 32768.0 * 4.0;
}

/**
 * @brief read raw temperature, gyroscope, and accelerometer values from LSM6DS3
 */
void lsm6ds3_read_raw_data(i2c_master_dev_handle_t dev_handle, lsm6ds3_data_t *data)
{
    /* read temperature if available*/
    if (lsm6ds3_check_temp_data_available(dev_handle))
    {
        lsm6ds3_read_temperature(dev_handle, &data->temp);
    }
    /* read gyroscope if new data available */
    if (lsm6ds3_check_gyro_data_available(dev_handle))
    {
        lsm6ds3_read_gyroscope(dev_handle, data->gyro);
    }

    /* read gyroscope if new data available */
    if (lsm6ds3_check_accel_data_available(dev_handle))
    {
        lsm6ds3_read_accelerometer(dev_handle, data->accel);
    }
}

/**
 * @brief read data from the FIFO of the LSM6DS3
 */

/* read-modify-write
1. First read the register,
2. Modify the register
3. write the register */
void read_modify_write(uint8_t bit_start,
                       uint8_t bit_end,
                       uint8_t field_value,
                       uint8_t *reg)
{
    uint8_t bit_len = bit_start - bit_end + 1;

    // /* create a mask of ones for the field of interest */
    // mask = ((1 << bit_len) - 1) << bit_end;
    // /* zero the field in the value using the mask*/
    // reg &= ~mask;
    // /* set the value of the field in the register */
    // reg |= (field_value & ((1 << bit_len) - 1)) << bit_end;

    /* one-liner */
    *reg = (*reg & ~(((1 << bit_len) - 1) << bit_end)) | ((field_value & ((1 << bit_len) - 1)) << bit_end);
}

uint8_t read_bit(uint8_t reg, uint8_t bit_pos)
{
    return reg & (1 << bit_pos);
}

void write_bit(uint8_t *reg, uint8_t bit_pos, uint8_t value)
{
    /* reset the bit first then set it if true */
    *reg = (*reg & ~(1 << bit_pos)) | (value << bit_pos);
}
