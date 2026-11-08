#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"

#define I2C_MASTER_SCL_IO           22        // Pin zegara (SCL)
#define I2C_MASTER_SDA_IO           21        // Pin danych (SDA)
#define I2C_MASTER_FREQ_HZ          100000    //  100 kHz
#define I2C_MASTER_DEVICE_ADDRESS    0x40    // domyslny adres dla modulu pca9865
// konfigurujemy I2C 
// zaczynamy od master busa
// master bus to główny bus I2C, na którym są podłączone urządzenia
// w tym przypadku jest to bus I2C1
// scl_io_num to pin, na którym jest podłączony scl
// sda_io_num to pin, na którym jest podłączony sda
// glitch_ignore_cnt to liczba cykli glitch, które mają być ignorowane
// enable_internal_pullup to flaga, która włącza wewnętrzny pullup
// na pinach scl i sda
// clk_source to źródło zegara, w tym przypadku jest to zegar systemowy
i2c_master_bus_config_t i2c_mst_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT, // zegar systemowy
    .i2c_port = -1, // -1 oznacza, że ma być użyty domyślny port I2C
    .scl_io_num = I2C_MASTER_SCL_IO, // pin 22
    .sda_io_num = I2C_MASTER_SDA_IO, // pin 21
    .glitch_ignore_cnt = 7, // filtorwanie zaklocen, standardowa wartosc dla esp32
    .flags.enable_internal_pullup = true, // rezystory pull up na pinach scl i sda
}; // konfiguracja master busa

//konfiguracja urzadzenia (slave)
i2c_device_config_t dev_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7, // 7 bitowy adres dla modulu pca9865
    .device_address = I2C_MASTER_DEVICE_ADDRESS, // domyslny adres dla modulu pca9865 
    .scl_speed_hz = I2C_MASTER_FREQ_HZ, // szybkosc i2c, standardowa wartosc dla esp32
};

i2c_master_bus_handle_t bus_handle; //uchwyt magistrali I2c
i2c_master_dev_handle_t dev_handle; // uchwyt urzadzenia dla modulu pca9865

void i2c_init(void){
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));
    ESP_LOGI(TAG, "I2C initialized");
}

// modul pca9685 ma rozdzielczosc 12 bitowa czyli 4095 pozycji
// pelny cykl to 50 hz czyli 20 ms
// pozycja srodkowa dla ser analogowych wymaga podania stanu wysokiego przez
// dokladnie 1.5 ms
// czyli pozadana liczba krokow to 1.5 ms / 20 ms * 4095 = 307
void reset_servos_position(void){
    // uint8_t ma dokladnie 8 bitow
    // pierwszy bit to jest rejestr urzadzenia a drugi to komenda
    uint8_t buf[2] = {0x00, 0x00};
    // ustawiamy bit uspienia (0x10)
    // uspienie jest wymagane dla zmiany czestotliwosci 
    buf[0] = 0x00;
    buf[1] = 0x10;
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, buf, 2, -1));
    // chwila delayu zeby modul mogl przetworzyc komende
    vTaskDelay(pdMS_TO_TICKS(10));
    // ustawiamy prescale 
    // ustawiamy 50hz bo tyle potrzebujemy do serw
    buf[0] = 0xFE; buf[1] = 0x79;
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, buf, 2, -1));
    vTaskDelay(pdMS_TO_TICKS(10));
    // wlaczamy auto-inkrementacja
    buf[0] = 0x00; buf[1] = 0x20;
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, buf, 2, -1));
    vTaskDelay(pdMS_TO_TICKS(10));
    // ustawiamy pozycji srodka (resetu) dla wszystkich serw naraz
    uint16_t off_val = 307;
    uint8_t pwn_data[5];

    pwn_data[0] = 0xFA; // adres poczatkowy
    pwn_data[1] = 0; // ON_L
    pwn_data[2] = 0; // ON_H
    pwn_data[3] = off_val & 0xFF; // OFF_L wyciagamy dolne 8 bitow 
    pwn_data[4] = off_val >> 8; // OFF_H wyciagamy gorne 8 bitow
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, pwn_data, 5, -1));
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_LOGI(TAG, "RESET WYKONANY");
    // wlaczamy serw
}   

#define BLINK_GPIO 2

static const char *TAG = "hello_world";

void app_main(void)
{
    ESP_LOGI(TAG, "uruchamianie esp32 z esp-idf...");
    int counter = 0;
    gpio_reset_pin(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
    ESP_LOGI(TAG, "Uruchamianie ESP32 z ESP-IDF...");
    
    i2c_init();             // odpalamy master bus magistrale I2C
    reset_servos_position(); // resetujemy pozycje serw
    while (1)
    {
        ESP_LOGI(TAG, "Hello world from ESP32! count=%d", counter++);
        gpio_set_level(BLINK_GPIO, 1);
        ESP_LOGI(TAG, "LED ON");
        vTaskDelay(pdMS_TO_TICKS(1000));
        gpio_set_level(BLINK_GPIO, 0);
        ESP_LOGI(TAG, "LED OFF");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
