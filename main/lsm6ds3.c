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
    read_modify_write(CTRL1_XL_FS_XL1, CTRL1_XL_FS_XL0, CTRL1_XL_FS_4G, &data[0]);

    /* set accelerometer ODR
    ODR_XL = 0b0100 for 104 Hz ODR */
    read_modify_write(CTRL1_XL_ODR_XL3, CTRL1_XL_ODR_XL0, CTRL1_XL_ODR_208HZ, &data[0]);

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
    read_modify_write(CTRL2_G_FS_G1, CTRL2_G_FS_G0, CTRL2_G_FS_1000DPS, &data[0]);

    /* set gyroscope ODR
    ODR_G = 0b0100 for 104 Hz ODR*/
    read_modify_write(CTRL2_G_ODR_G3, CTRL2_G_ODR_G0, CTRL2_G_ODR_208HZ, &data[0]);

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
    g[0] = g_raw[0] / 32768.0 * 1000;
    g[1] = g_raw[1] / 32768.0 * 1000;
    g[2] = g_raw[2] / 32768.0 * 1000;
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
 * @brief set up the FIFO of the LSM6DS3
 */
void lsm6ds3_fifo_init(i2c_master_dev_handle_t dev_handle)
{
    /* enable block data update (BDU) */
    ESP_ERROR_CHECK(lsm6ds3_register_read(dev_handle, LSM6DS3_CTRL3_C_REG_ADDR, data, 1));
    read_modify_write(CTRL3_C_BDU, CTRL3_C_BDU, 1, data);
    ESP_ERROR_CHECK(lsm6ds3_register_write_byte(dev_handle, LSM6DS3_CTRL3_C_REG_ADDR, data[0]));

    /* set FIFO threshold */
    data[0] = 0;
    data[1] = 0;
    uint16_t fifo_threshold = 1000;
    read_modify_write(FIFO_CTRL1_FTH_7, FIFO_CTRL1_FTH_0, fifo_threshold & 0xFF, &data[0]);
    //
    read_modify_write(FIFO_CTRL2_FTH_10, FIFO_CTRL2_FTH_8, (fifo_threshold >> 8) & 0x7, &data[1]);

    /* include temperature in FIFO */
    read_modify_write(FIFO_CTRL2_TEMP_EN, FIFO_CTRL2_TEMP_EN, 0x1, &data[1]);
    ESP_ERROR_CHECK(lsm6ds3_register_write_byte(dev_handle, LSM6DS3_FIFO_CTRL1_REG_ADDR, data[0]));
    ESP_ERROR_CHECK(lsm6ds3_register_write_byte(dev_handle, LSM6DS3_FIFO_CTRL2_REG_ADDR, data[1]));

    // ESP_ERROR_CHECK(lsm6ds3_register_read(dev_handle, LSM6DS3_FIFO_CTRL1_REG_ADDR, data, 2));
    ESP_ERROR_CHECK(lsm6ds3_register_read(dev_handle, LSM6DS3_FIFO_CTRL1_REG_ADDR, &data[0], 1));
    ESP_ERROR_CHECK(lsm6ds3_register_read(dev_handle, LSM6DS3_FIFO_CTRL2_REG_ADDR, &data[1], 1));
    ESP_LOGI(LSM6DS3_TAG, "FIFO_CTRL1_REG_ADDR = %X", data[0]);
    ESP_LOGI(LSM6DS3_TAG, "FIFO_CTRL2_REG_ADDR = %X", data[1]);

    /* set gyroscope decimation rate
    no decimation */
    data[0] = 0;
    read_modify_write(FIFO_CTRL3_DEC_GYRO2, FIFO_CTRL3_DEC_GYRO0, 0x1, &data[0]);

    /* set accelerometer decimation rate */
    read_modify_write(FIFO_CTRL3_DEC_ACCEL2, FIFO_CTRL3_DEC_ACCEL0, 0x1, &data[0]);

    ESP_ERROR_CHECK(lsm6ds3_register_write_byte(dev_handle, LSM6DS3_FIFO_CTRL3_REG_ADDR, data[0]));

    /* disable 3rd FIFO data set and enable 4th FIFO data set (for temperature) */
    data[0] = 0;
    // read_modify_write(FIFO_CTRL4_STOP_ON_FTH, FIFO_CTRL4_STOP_ON_FTH, 0x1, &data[0]);
    // read_modify_write(FIFO_CTRL4_DEC_DS4_FIFO2, FIFO_CTRL4_DEC_DS4_FIFO0, 0x1, &data[0]);
    // read_modify_write(FIFO_CTRL4_DEC_DS3_FIFO2, FIFO_CTRL4_DEC_DS3_FIFO0, 0x1, &data[0]);

    ESP_ERROR_CHECK(lsm6ds3_register_write_byte(dev_handle, LSM6DS3_FIFO_CTRL4_REG_ADDR, data[0]));

    /* Set FIFO ODR and FIFO mode
    FIFO ODR = 0b0100 for 104 Hz */
    data[0] = 0;
    read_modify_write(FIFO_CTRL5_ODR_3, FIFO_CTRL5_ODR_0, FIFO_CTRL5_ODR_FIFO_208HZ, &data[0]);
    ESP_ERROR_CHECK(lsm6ds3_register_write_byte(dev_handle, LSM6DS3_FIFO_CTRL5_REG_ADDR, data[0]));
}

/**
 * @brief disable FIFO of the LSM6DS3
 */
void lsm6ds3_fifo_stop(i2c_master_dev_handle_t dev_handle)
{
    /* reset FIFO by setting it to bypass mode
    FIFO mode = 0b000 for bypass mode */
    data[0] = 0;
    ESP_ERROR_CHECK(lsm6ds3_register_read(dev_handle, LSM6DS3_FIFO_CTRL5_REG_ADDR, &data[0], 1));
    read_modify_write(FIFO_CTRL5_MODE_2, FIFO_CTRL5_MODE_0, FIFO_CTRL5_MODE_BYPASS, &data[0]);
    ESP_ERROR_CHECK(lsm6ds3_register_write_byte(dev_handle, LSM6DS3_FIFO_CTRL5_REG_ADDR, data[0]));
}

/**
 * @brief start recording data samples to the FIFO of the LSM6DS3
 */
void lsm6ds3_fifo_reset_start(i2c_master_dev_handle_t dev_handle)
{
    /* read FIFO CTRL */
    // ESP_ERROR_CHECK(lsm6ds3_register_read(dev_handle, LSM6DS3_FIFO_CTRL1_REG_ADDR, &data[0], 1));
    // ESP_ERROR_CHECK(lsm6ds3_register_read(dev_handle, LSM6DS3_FIFO_CTRL2_REG_ADDR, &data[1], 1));
    // ESP_LOGI(LSM6DS3_TAG, "FIFO_CTRL1_REG_ADDR = %X", data[0]);
    // ESP_LOGI(LSM6DS3_TAG, "FIFO_CTRL2_REG_ADDR = %X", data[1]);

    /* reset FIFO */
    lsm6ds3_fifo_stop(dev_handle);

    /* start FIFO in FIFO mode
    FIFO mode = 0b001 for FIFO mode*/
    ESP_ERROR_CHECK(lsm6ds3_register_read(dev_handle, LSM6DS3_FIFO_CTRL5_REG_ADDR, &data[0], 1));
    read_modify_write(FIFO_CTRL5_MODE_2, FIFO_CTRL5_MODE_0, FIFO_CTRL5_MODE_FIFO, &data[0]);
    ESP_ERROR_CHECK(lsm6ds3_register_write_byte(dev_handle, LSM6DS3_FIFO_CTRL5_REG_ADDR, data[0]));
    // ESP_ERROR_CHECK(lsm6ds3_register_read(dev_handle, LSM6DS3_FIFO_CTRL5_REG_ADDR, &data[0], 1));
    // ESP_LOGI(LSM6DS3_TAG, "FIFO_CTRL5_REG_ADDR = %X", data[0]);
}

/**
 * @brief read data from the FIFO of the LSM6DS3 at a fixed frequency
 *
 * Read
 * , lsm6ds3_data_t *lsm6ds3_fifo_buffer
 */
void lsm6ds3_fifo_read(i2c_master_dev_handle_t dev_handle, lsm6ds3_data_t *lsm6ds3_fifo_buffer, uint16_t num_timestamps)
{
    uint16_t num_fifo_samples;
    uint16_t fifo_pattern, fifo_data;

    /**
     * The accelerometer and gyroscope both log at 104 Hz.
     * The temperature sensor logs at 52 Hz.
     * Each group is 6 bytes
     *
     * 1st 6-byte group is gyroscope
     * 2nd group is accelerometer
     *
     * t = 0,
     */
    // /* read FIFO CTRL again */
    // ESP_ERROR_CHECK(lsm6ds3_register_read(dev_handle, LSM6DS3_FIFO_CTRL1_REG_ADDR, &data[0], 1));
    // ESP_ERROR_CHECK(lsm6ds3_register_read(dev_handle, LSM6DS3_FIFO_CTRL2_REG_ADDR, &data[1], 1));
    // ESP_LOGI(LSM6DS3_TAG, "FIFO_CTRL1_REG_ADDR again = %X", data[0]);
    // ESP_LOGI(LSM6DS3_TAG, "FIFO_CTRL2_REG_ADDR again = %X", data[1]);

    // do
    // {
    //     /* get number of FIFO samples */
    //     ESP_ERROR_CHECK(lsm6ds3_register_read(dev_handle, LSM6DS3_FIFO_STATUS1_REG_ADDR, data, 2));
    //     // ESP_ERROR_CHECK(lsm6ds3_register_read(dev_handle, LSM6DS3_FIFO_STATUS2_REG_ADDR, &data[1], 1));
    //     num_fifo_samples = ((data[1] & 0x7) << 8 | data[0]);
    //     ESP_LOGI(LSM6DS3_TAG, "num_fifo_samples = %d", num_fifo_samples);
    // } while (num_fifo_samples < 40);

    // // grab 40 samples from the FIFO and print out the FIFO pattern
    // while (num_fifo_samples > 0)
    // {
    //     /* get FIFO pattern */
    //     ESP_ERROR_CHECK(lsm6ds3_register_read(dev_handle, LSM6DS3_FIFO_STATUS3_REG_ADDR, data, 2));
    //     // ESP_ERROR_CHECK(lsm6ds3_register_read(dev_handle, LSM6DS3_FIFO_STATUS4_REG_ADDR, &data[1], 1));
    //     fifo_pattern = ((data[1] & 0x3) << 8 | data[0]);
    //     ESP_LOGI(LSM6DS3_TAG, "fifo_pattern = %X", fifo_pattern);

    //     /* read 16 bits from FIFO data out H and L */
    //     ESP_ERROR_CHECK(lsm6ds3_register_read(dev_handle, LSM6DS3_FIFO_DATA_OUT_L_REG_ADDR, data, 2));
    //     // ESP_ERROR_CHECK(lsm6ds3_register_read(dev_handle, LSM6DS3_FIFO_DATA_OUT_H_REG_ADDR, &data[1], 1));
    //     fifo_data = ((data[1] << 8) | data[0]);
    //     ESP_LOGI(LSM6DS3_TAG, "fifo_data = %X", fifo_data);

    //     num_fifo_samples--;
    // }

    ESP_ERROR_CHECK(lsm6ds3_register_read(dev_handle, LSM6DS3_FIFO_STATUS1_REG_ADDR, data, 2));
    num_fifo_samples = (((data[1] & 0x7) << 8) | data[0]);
    // ESP_LOGI(LSM6DS3_TAG, "num_fifo_samples = %d", num_fifo_samples);

    if (num_fifo_samples > num_timestamps * 6)
    {
        ESP_ERROR_CHECK(lsm6ds3_register_read(dev_handle, LSM6DS3_FIFO_STATUS1_REG_ADDR, data, 2));
        num_fifo_samples = (((data[1] & 0x7) << 8) | data[0]);

        for (int i = 0; i < num_timestamps; i++)
        {
            ESP_ERROR_CHECK(lsm6ds3_register_read(dev_handle, LSM6DS3_FIFO_DATA_OUT_L_REG_ADDR, data, 2));
            lsm6ds3_fifo_buffer[i].gyro[0] = ((int16_t)(data[1] << 8) | data[0]) / 32768.0 * 1000;
            ESP_ERROR_CHECK(lsm6ds3_register_read(dev_handle, LSM6DS3_FIFO_DATA_OUT_L_REG_ADDR, data, 2));
            lsm6ds3_fifo_buffer[i].gyro[1] = ((int16_t)(data[1] << 8) | data[0]) / 32768.0 * 1000;
            ESP_ERROR_CHECK(lsm6ds3_register_read(dev_handle, LSM6DS3_FIFO_DATA_OUT_L_REG_ADDR, data, 2));
            lsm6ds3_fifo_buffer[i].gyro[2] = ((int16_t)(data[1] << 8) | data[0]) / 32768.0 * 1000;
            ESP_ERROR_CHECK(lsm6ds3_register_read(dev_handle, LSM6DS3_FIFO_DATA_OUT_L_REG_ADDR, data, 2));
            lsm6ds3_fifo_buffer[i].accel[0] = ((int16_t)(data[1] << 8) | data[0]) / 32768.0 * 4.0;
            ESP_ERROR_CHECK(lsm6ds3_register_read(dev_handle, LSM6DS3_FIFO_DATA_OUT_L_REG_ADDR, data, 2));
            lsm6ds3_fifo_buffer[i].accel[1] = ((int16_t)(data[1] << 8) | data[0]) / 32768.0 * 4.0;
            ESP_ERROR_CHECK(lsm6ds3_register_read(dev_handle, LSM6DS3_FIFO_DATA_OUT_L_REG_ADDR, data, 2));
            lsm6ds3_fifo_buffer[i].accel[2] = ((int16_t)(data[1] << 8) | data[0]) / 32768.0 * 4.0;

            // vTaskDelay(10 / portTICK_PERIOD_MS);
            // ESP_LOGI(LSM6DS3_TAG, "fifo_data %d = %X", i, fifo_data);
        }

        // ESP_LOGI(LSM6DS3_TAG, "temperature = %f\n", lsm6ds3_data.temp);
        ESP_LOGI(LSM6DS3_TAG, "gyro = %f %f %f\n", lsm6ds3_fifo_buffer[num_timestamps - 1].gyro[0], lsm6ds3_fifo_buffer[num_timestamps - 1].gyro[1], lsm6ds3_fifo_buffer[num_timestamps - 1].gyro[2]);
        ESP_LOGI(LSM6DS3_TAG, "accel = %f %f %f\n", lsm6ds3_fifo_buffer[num_timestamps - 1].accel[0], lsm6ds3_fifo_buffer[num_timestamps - 1].accel[1], lsm6ds3_fifo_buffer[num_timestamps - 1].accel[2]);
        ESP_LOGI(LSM6DS3_TAG, "num_fifo_samples = %d", num_fifo_samples);
    }

    // /* check FIFO overrun */
    // read_bit(data[1], FIFO_STATUS2_OVER_RUN);

    // /* check FIFO full */
    // read_bit(data[1], FIFO_STATUS2_FIFO_FULL_SMART);

    // /* check if FIFO is empty */
    // read_bit(data[1], FIFO_STATUS2_FIFO_EMPTY);
}

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
