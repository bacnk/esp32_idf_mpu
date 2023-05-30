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
#define I2C_MASTER_SCL_IO 22 /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_NUM I2C_NUM_0    /*!< gpio number for I2C master data  */
#define I2C_MASTER_FREQ_HZ 100000   /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */

#define WRITE_BIT I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ   /*!< I2C master read */
#define ACK_CHECK_EN 0x1           /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0          /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                /*!< I2C ack value */
#define NACK_VAL 0x1               /*!< I2C nack value */

/*MPU6050 register addresses */

#define MPU6050_REG_POWER 0x6B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_GYRO_CONFIG 0x1B

/*These are the addresses of mpu6050 from which you will fetch accelerometer x,y,z high and low values */
#define MPU6050_REG_ACC_X_HIGH 0x3B
#define MPU6050_REG_ACC_X_LOW 0x3C
#define MPU6050_REG_ACC_Y_HIGH 0x3D
#define MPU6050_REG_ACC_Y_LOW 0x3E
#define MPU6050_REG_ACC_Z_HIGH 0x3F
#define MPU6050_REG_ACC_Z_LOW 0x40

/*These are the addresses of mpu6050 from which you will fetch gyro x,y,z high and low values */

#define MPU6050_REG_GYRO_X_HIGH 0x43
#define MPU6050_REG_GYRO_X_LOW 0x44
#define MPU6050_REG_GYRO_Y_HIGH 0x45
#define MPU6050_REG_GYRO_Y_LOW 0x46
#define MPU6050_REG_GYRO_Z_HIGH 0x47
#define MPU6050_REG_GYRO_Z_LOW 0x48

/*MPU6050 address and who am i register*/

#define MPU6050_SLAVE_ADDR 0x68
#define who_am_i 0x75
uint8_t buffer[14];
KalmanFilter_t kalmanFilterAccelX;
KalmanFilter_t kalmanFilterAccelY;
KalmanFilter_t kalmanFilterAccelZ;
KalmanFilter_t kalmanFilterGyroX;
KalmanFilter_t kalmanFilterGyroY;
KalmanFilter_t kalmanFilterGyroZ;
float filteredAccelX;
float filteredAccelY;
float filteredAccelZ;
float filteredGyroX;
float filteredGyroY;
float filteredGyroZ;
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
    i2c_master_write_byte(cmd, 0x00, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t i2c_master_sensor_test(uint8_t length, uint8_t *data, uint16_t timeout)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU6050_SLAVE_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, MPU6050_REG_ACC_X_HIGH, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    vTaskDelay(30 / portTICK_RATE_MS);
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU6050_SLAVE_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read(cmd, data, length - 1, ACK_VAL);
    i2c_master_read(cmd, data, 1, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}
float convertAccel(int16_t accelValue)
{
    // Áp dụng công thức chuyển đổi từ giá trị đọc sang tọa độ gia tốc
    float accel = accelValue / 16384.0; // Ví dụ: Tỷ lệ chuyển đổi cho accelerometer

    return accel;
}
float convertGyro(int16_t gyroValue)
{
    // Áp dụng công thức chuyển đổi từ giá trị đọc sang tọa độ gióc
    float gyro = gyroValue / 131.0; // Ví dụ: Tỷ lệ chuyển đổi cho gyroscope

    return gyro;
}
float convertTemperature(int16_t tempValue)
{
    // Áp dụng công thức chuyển đổi từ giá trị đọc sang nhiệt độ
    float temperature = (tempValue / 340.0) + 36.53; // Ví dụ: Công thức chuyển đổi cho MPU6050

    return temperature;
}
static void disp_buf(uint8_t *buf, int len)
{
    int i;
    uint16_t pbuffer[7];
    pbuffer[0] = (int16_t)((buf[0] << 8) | buf[1]);
    float accelX = convertAccel(pbuffer[0]);
    pbuffer[1] = (int16_t)((buf[2] << 8) | buf[3]);
    float accelY = convertAccel(pbuffer[1]);
    pbuffer[2] = (int16_t)((buf[4] << 8) | buf[5]);
    float accelZ = convertAccel(pbuffer[2]);
    pbuffer[3] = (int16_t)((buf[6] << 8) | buf[7]);
    float temperature = convertTemperature(pbuffer[3]);

    pbuffer[4] = (int16_t)((buf[8] << 8) | buf[9]);
    float gyroX = convertGyro(pbuffer[4]);
    pbuffer[5] = (int16_t)((buf[10] << 8) | buf[11]);
    float gyroY = convertGyro(pbuffer[5]);
    pbuffer[6] = (int16_t)((buf[12] << 8) | buf[13]);
    float gyroZ = convertGyro(pbuffer[6]);
    filteredAccelX = kalmanFilterUpdate(&kalmanFilterAccelX, accelX);
    filteredAccelY = kalmanFilterUpdate(&kalmanFilterAccelY, accelY);
    filteredAccelZ = kalmanFilterUpdate(&kalmanFilterAccelZ, accelZ);

    // Apply Kalman filter to gyroscope data
    filteredGyroX = kalmanFilterUpdate(&kalmanFilterGyroX, gyroX);
    filteredGyroY = kalmanFilterUpdate(&kalmanFilterGyroY, gyroY);
    filteredGyroZ = kalmanFilterUpdate(&kalmanFilterGyroZ, gyroZ);
    // In ra giá trị đã lọc bằng bộ lọc Kalman

    ESP_LOGI(TAG, "Accelerometer:  X=%.2f  m/s^2  Y=%.2f m/s^2 Z=%.2f m/s^2\n", accelX, accelY, accelZ);
    ESP_LOGI(TAG, "Temperature: %.2f\n", temperature);
    ESP_LOGI(TAG, "Gyroscope:  X=%.2f radians/s  Y=%.2f  radians/s Z=%.2f radians/s\n", gyroX, gyroY, gyroZ);
    ESP_LOGI(TAG, "Filtered Accelerometer:  X=%.2f Y=%.2f Z=%.2f\n", filteredAccelX, filteredAccelY, filteredAccelZ);
    ESP_LOGI(TAG, "Filtered Gyroscope:  X=%.2f Y=%.2f Z=%.2f\n", filteredGyroX, filteredGyroY, filteredGyroZ);
}

void send_data_to_nodejs(float filteredAccelX, float filteredAccelY, float filteredAccelZ, float filteredGyroX, float filteredGyroY, float filteredGyroZ)
{
    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "filteredAccelX", filteredAccelX);
    cJSON_AddNumberToObject(root, "filteredAccelY", filteredAccelY);
    cJSON_AddNumberToObject(root, "filteredAccelZ", filteredAccelZ);
    cJSON_AddNumberToObject(root, "filteredGyroX", filteredGyroX);
    cJSON_AddNumberToObject(root, "filteredGyroY", filteredGyroY);
    cJSON_AddNumberToObject(root, "filteredGyroZ", filteredGyroZ);
    char *json_data = cJSON_Print(root);
    int sockfd;
    struct sockaddr_in dest_addr;
    struct hostent *he;
    int numbytes;

    // Create socket
    if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1)
    {
        perror("socket");
        return;
    }

    // Set server information
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(8080);  // Change to the appropriate port number
    he = gethostbyname("10.10.18.65"); // Change to the server hostname or IP address
    dest_addr.sin_addr = *((struct in_addr *)he->h_addr);
    memset(&(dest_addr.sin_zero), '\0', 8);

    // Connect to server
    if (connect(sockfd, (struct sockaddr *)&dest_addr, sizeof(struct sockaddr)) == -1)
    {
        ESP_LOGD("connect");
        return;
    }

    // Send data
    if ((numbytes = send(sockfd, json_data, strlen(json_data), 0)) == -1)
    {
        perror("send");
        return;
    }

    printf("Sent %d bytes to server\n", numbytes);

    // Close socket
    close(sockfd);

    cJSON_Delete(root);
    free(json_data);
}

static void i2c_task(void *arg)
{
    int ret;

    while (1)
    {
        ret = i2c_master_sensor_test(14, &buffer[0], 0);
        if (ret == ESP_ERR_TIMEOUT)
        {
            printf("\n I2C Timeout");
        }
        else if (ret == ESP_OK)
        {
            printf("*******************\n");
            printf("TASK: MASTER READ SENSOR( MPU6050 )\n");
            printf("*******************\n");
        }
        else
        {
            printf("\n No ack, sensor not connected...skip...");
        }
        vTaskDelay(500 / portTICK_RATE_MS);
        disp_buf(&buffer[0], 14);
        send_data_to_nodejs(filteredAccelX, filteredAccelY, filteredAccelZ, filteredGyroX, filteredGyroY, filteredGyroZ);
        //  vTaskDelay(30 / portTICK_RATE_MS);
    }
    vTaskDelete(NULL);
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_ERROR_CHECK(mpu_wake());
    connect_wifi();
    // Khởi tạo bộ lọc Kalman cho tọa độ gia tốc X
    // Khởi tạo bộ lọc Kalman cho tọa độ gia tốc X
    // Khởi tạo bộ lọc Kalman cho tọa độ gia tốc X

    kalmanFilterInit(&kalmanFilterAccelX, 1, 1);
    kalmanFilterInit(&kalmanFilterAccelY, 1, 1);
    kalmanFilterInit(&kalmanFilterAccelZ, 1, 1);
    kalmanFilterInit(&kalmanFilterGyroX, 1, 1);
    kalmanFilterInit(&kalmanFilterGyroY, 1, 1);
    kalmanFilterInit(&kalmanFilterGyroZ, 1, 1);

    xTaskCreate(i2c_task, "i2c_test_task", 1024 * 2, NULL, 10, NULL);
}