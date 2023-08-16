#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/Task.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "time.h"

#define I2C_SLAVE_ADDR 0x68
#define I2C_SDA 4
#define I2C_SCL 5

static const char *TAG = "ds1307";

static esp_err_t set_i2c(void)
{
    i2c_config_t i2c_config = {};
    i2c_config.mode = I2C_MODE_MASTER;
    i2c_config.sda_io_num = I2C_SDA;
    i2c_config.scl_io_num = I2C_SCL;
    i2c_config.sda_pullup_en = true;
    i2c_config.scl_pullup_en = true;
    i2c_config.master.clk_speed = 400000;
    i2c_config.clk_flags = 0;

    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &i2c_config));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, ESP_INTR_FLAG_LEVEL1));

    return ESP_OK;
}

static esp_err_t ds1307_init(void)
{
    //Manual date and time assignment

    uint8_t init_data[] = {
        0x23, // Seconds
        0x59, // Minutes
        0x23, // Hours (24 hour format)
        0x06, // Day of the week (6=Saturday)
        0x21, // Day of the mount
        0x08, // Month
        0x21  // Year (2021 in two-digit format)
    };
    //Sending address ds1307 and data 
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();                                    // Crear un manejador de comandos I2C
    i2c_master_start(cmd);                                                           // Agregar un comando de inicio de comunicación I2C
    i2c_master_write_byte(cmd, (I2C_SLAVE_ADDR << 1) | I2C_MASTER_WRITE, true);      // Escribir dirección del DS1307 en modo escritura
    i2c_master_write_byte(cmd, 0x00, 0x1);                                           // Comenzar en el registro de segundos
    i2c_master_write(cmd, init_data, sizeof(init_data), true);                       // Escribir los valores de inicialización
    i2c_master_stop(cmd);                                                            // Agregar un comando de detención de comunicación I2C
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS); // Ejecutar la secuencia de comandos I2C
    i2c_cmd_link_delete(cmd);                                                        // Liberar el manejador de comandos
    //Error checking
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "Comunicacion i2c completa");
    }
    else if (ret == ESP_ERR_TIMEOUT)
    {
        ESP_LOGE(TAG, "No envio a tiempo la configuracion");
    }
    else
    {
        ESP_LOGE(TAG, "Error en la comunicacion");
    }

    return ESP_OK;
}

static esp_err_t ds1307_get_time(void){
    uint8_t rx_data[7];
    uint8_t command = 0x00;
    while (1)
    {
        i2c_master_write_read_device(I2C_NUM_0,
                                     I2C_SLAVE_ADDR,
                                     &command,
                                     1,
                                     rx_data,
                                     7,
                                     pdMS_TO_TICKS(1000));

        // Imprimir la fecha y hora en la pantalla
        printf("Fecha y Hora Actual:\n");
        printf("   %02x:%02x:%02x\n", rx_data[2], rx_data[1], rx_data[0]);   // Hora:Minutos:Segundos
        printf("   %02x/%02x/%02x\n", rx_data[4], rx_data[5], rx_data[6]); // Día/Mes/Año

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    return ESP_OK;
}

void app_main()
{
    ESP_ERROR_CHECK(set_i2c());
    ESP_ERROR_CHECK(ds1307_init());
    ESP_ERROR_CHECK(ds1307_get_time());
}