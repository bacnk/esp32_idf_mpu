#include <stdio.h>
#include "driver/i2c.h"
#include <stdlib.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "KalmanFilter.h"
#include <lwip/sockets.h>
#include <lwip/netdb.h>
#include "esp_system.h"
#include <cJSON.h>
#include "connect_wifi.h"

static const char *TAG = "MPU_6050";
static const char *TAG1 = "HTTP_POST";

#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0
#define WRITE_BIT I2C_MASTER_WRITE
#define READ_BIT I2C_MASTER_READ
#define ACK_CHECK_EN 0x1
#define ACK_CHECK_DIS 0x0
#define ACK_VAL 0x0
#define NACK_VAL 0x1

#define MPU6050_REG_POWER 0x6B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_GYRO_CONFIG 0x1B

#define MPU6050_REG_ACC_X_HIGH 0x3B
#define MPU6050_REG_ACC_X_LOW 0x3C
#define MPU6050_REG_ACC_Y_HIGH 0x3D
#define MPU6050_REG_ACC_Y_LOW 0x3E
#define MPU6050_REG_ACC_Z_HIGH 0x3F
#define MPU6050_REG_ACC_Z_LOW 0x40

#define MPU6050_TEMP_OUT_H 0X41
#define MPU6050_REG_GYRO_X_HIGH 0x43
#define MPU6050_REG_GYRO_X_LOW 0x44
#define MPU6050_REG_GYRO_Y_HIGH 0x45
#define MPU6050_REG_GYRO_Y_LOW 0x46
#define MPU6050_REG_GYRO_Z_HIGH 0x47
#define MPU6050_REG_GYRO_Z_LOW 0x48

#define MPU6050_SLAVE_ADDR 0x68
#define who_am_i 0x75
#define ESP_CONFIG_TCP_SERVER CONFIG_SERVER_IP
#define ESP_CONFIG_TCP_PORT CONFIG_SERVER_PORT
static uint8_t buffer[14];
static KalmanFilter_t kalmanFilterAccelX;
static KalmanFilter_t kalmanFilterAccelY;
static KalmanFilter_t kalmanFilterAccelZ;
static KalmanFilter_t kalmanFilterGyroX;
static KalmanFilter_t kalmanFilterGyroY;
static KalmanFilter_t kalmanFilterGyroZ;

static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        .clk_flags = 0,
    };
    i2c_param_config(I2C_NUM_0, &conf);
    return i2c_driver_install(I2C_NUM_0, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

static esp_err_t mpu_wake(void)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU6050_SLAVE_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, MPU6050_REG_POWER, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t mpu_read_accel(uint16_t *accel_x, uint16_t *accel_y, uint16_t *accel_z)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU6050_SLAVE_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, MPU6050_REG_ACC_X_HIGH, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    if (ret != ESP_OK)
    {
        return ret;
    }
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU6050_SLAVE_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, (uint8_t *)accel_x, ACK_VAL);
    i2c_master_read_byte(cmd, ((uint8_t *)accel_x) + 1, ACK_VAL);
    i2c_master_read_byte(cmd, (uint8_t *)accel_y, ACK_VAL);
    i2c_master_read_byte(cmd, ((uint8_t *)accel_y) + 1, ACK_VAL);
    i2c_master_read_byte(cmd, (uint8_t *)accel_z, ACK_VAL);
    i2c_master_read_byte(cmd, ((uint8_t *)accel_z) + 1, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t mpu_read_gyro(uint16_t *gyro_x, uint16_t *gyro_y, uint16_t *gyro_z)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU6050_SLAVE_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, MPU6050_REG_GYRO_X_HIGH, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    if (ret != ESP_OK)
    {
        return ret;
    }
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU6050_SLAVE_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, (uint8_t *)gyro_x, ACK_VAL);
    i2c_master_read_byte(cmd, ((uint8_t *)gyro_x) + 1, ACK_VAL);
    i2c_master_read_byte(cmd, (uint8_t *)gyro_y, ACK_VAL);
    i2c_master_read_byte(cmd, ((uint8_t *)gyro_y) + 1, ACK_VAL);
    i2c_master_read_byte(cmd, (uint8_t *)gyro_z, ACK_VAL);
    i2c_master_read_byte(cmd, ((uint8_t *)gyro_z) + 1, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}
static esp_err_t mpu_read_temperature(float *temperature)
{
    esp_err_t ret;

    uint8_t temp_data[2];
    int16_t raw_temperature;
    // Gửi lệnh để đọc dữ liệu nhiệt độ từ MPU6050
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU6050_SLAVE_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, MPU6050_TEMP_OUT_H, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    if (ret != ESP_OK)
    {
        i2c_cmd_link_delete(cmd);
        return ret;
    }

    // Đọc dữ liệu nhiệt độ từ MPU6050
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU6050_SLAVE_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, temp_data, ACK_VAL);
    i2c_master_read_byte(cmd, temperature + 1, ACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK)
    {
        return ret;
    }

    // Chuyển đổi dữ liệu nhiệt độ sang giá trị số nguyên

    raw_temperature = (int16_t)((temp_data[0] << 8) | temp_data[1]);

    // Apply the temperature conversion formula
    *temperature = (float)raw_temperature / 340.0 + 36.53;
    return ESP_OK;
}

static esp_err_t init_mpu6050(void)
{
    ESP_LOGI(TAG, "Initializing MPU6050");

    if (i2c_master_init() != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize I2C");
        return ESP_FAIL;
    }

    if (mpu_wake() != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to wake MPU6050");
        return ESP_FAIL;
    }

    kalmanFilterInit(&kalmanFilterAccelX, 1, 1);
    kalmanFilterInit(&kalmanFilterAccelY, 1, 1);
    kalmanFilterInit(&kalmanFilterAccelZ, 1, 1);
    kalmanFilterInit(&kalmanFilterGyroX, 1, 1);
    kalmanFilterInit(&kalmanFilterGyroY, 1, 1);
    kalmanFilterInit(&kalmanFilterGyroZ, 1, 1);

    return ESP_OK;
}
float convertAccel(int16_t accelValue)
{
    // Áp dụng công thức chuyển đổi từ giá trị đọc sang tọa độ gia tốc
    float accel = accelValue / 16384.0; // Tỷ lệ chuyển đổi cho accelerometer

    return accel;
}

float convertGyro(int16_t gyroValue)
{
    // Áp dụng công thức chuyển đổi từ giá trị đọc sang tọa độ gióc
    float gyro = gyroValue / 131.0; // Tỷ lệ chuyển đổi cho gyroscope

    return gyro;
}

void app_main()
{
    ESP_ERROR_CHECK(nvs_flash_init());
    connect_wifi();
    if (init_mpu6050() != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize MPU6050");
        return;
    }

    uint16_t accel_x, accel_y, accel_z;
    uint16_t gyro_x, gyro_y, gyro_z;
    float temperature;
    cJSON *root, *acceleration, *gyro, *temp;
    char *json_data;

    while (1)
    {
        if (mpu_read_accel(&accel_x, &accel_y, &accel_z) == ESP_OK && mpu_read_gyro(&gyro_x, &gyro_y, &gyro_z) == ESP_OK && mpu_read_temperature(&temperature) == ESP_OK)
        {
            // Kalman filter for accelerometer data
            float filtered_accel_x = convertAccel(kalmanFilterUpdate(&kalmanFilterAccelX, accel_x));
            float filtered_accel_y = convertAccel(kalmanFilterUpdate(&kalmanFilterAccelY, accel_y));
            float filtered_accel_z = convertAccel(kalmanFilterUpdate(&kalmanFilterAccelZ, accel_z));

            // Kalman filter for gyroscope data
            float filtered_gyro_x = convertGyro(kalmanFilterUpdate(&kalmanFilterGyroX, gyro_x));
            float filtered_gyro_y = convertGyro(kalmanFilterUpdate(&kalmanFilterGyroY, gyro_y));
            float filtered_gyro_z = convertGyro(kalmanFilterUpdate(&kalmanFilterGyroZ, gyro_z));

            root = cJSON_CreateObject();
            cJSON_AddItemToObject(root, "acceleration", acceleration = cJSON_CreateObject());
            cJSON_AddNumberToObject(acceleration, "x", filtered_accel_x);
            cJSON_AddNumberToObject(acceleration, "y", filtered_accel_y);
            cJSON_AddNumberToObject(acceleration, "z", filtered_accel_z);
            cJSON_AddItemToObject(root, "gyro", gyro = cJSON_CreateObject());
            cJSON_AddNumberToObject(gyro, "x", filtered_gyro_x);
            cJSON_AddNumberToObject(gyro, "y", filtered_gyro_y);
            cJSON_AddNumberToObject(gyro, "z", filtered_gyro_z);
            cJSON_AddItemToObject(root, "temperature", temp = cJSON_CreateObject());
            cJSON_AddNumberToObject(temp, "TEMP", temperature);
            json_data = cJSON_Print(root);
            cJSON_Delete(root);

            ESP_LOGI(TAG1, "JSON data: %s", json_data);
            free(json_data);
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
