#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include <math.h>

#define I2C_MASTER_SCL_IO 22           // Pin zegara (SCL)
#define I2C_MASTER_SDA_IO 21           // Pin danych (SDA)
#define I2C_MASTER_FREQ_HZ 100000      //  100 kHz
#define I2C_MASTER_DEVICE_ADDRESS 0x40 // domyslny adres dla modulu pca9865

// define dla fizycznych wymiarow robota
// narazie sa tam wrzucone losowe liczby

// w konwencji dh nie mozna robic pustych macierzy ktore nie dotycza przegubow
// dlatego zakladamy ze serwo 1 jest tak jakby w stole
// bedzie to pozwalalo na to ze robot bedzie wiedzial gdzie jest Z = absolutne 0 (stol)
#define L1_BASE_HEIGHT 60.0f    // d1: od stolu do serwa 2
#define L2_ARM1_LENGTH 120.0f   // a2: od serwa 2 do serwa 3
#define L3_ARM2_LENGTH 130.0f   // a3: od serwa 3 do serwa 4
#define L4_GRIPPER_OFFSET 50.0f // a4: od serwa 4 do konca chwytaka

// definiujemy wartosc pi do konwersji na radiany jak jej nie ma
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

//  struct do trzymania informacji pojedynczego serwa/ramienia

typedef struct
{
    float theta;
    float d;
    float a;
    float alpha;
} SingleArm;

SingleArm robot_joints[4]; // pelne 4 osie

// generyczna funkcja do mnozenia macierzy 4x4
void multiply_matrices(float A[4][4], float B[4][4], float result[4][4])
{
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            float subResult = 0;
            for (int counter = 0; counter < 4; counter++)
            {
                float multiplication = A[i][counter] * B[counter][j];
                subResult = subResult + multiplication;
            }
            result[i][j] = subResult;
        }
    }
}
// funkcja do generowania macierzy dh
void generate_dh_matrix(SingleArm arm, float result[4][4])
{
    // c przyjmuje radiany wiec zamieniamy katy
    float th = arm.theta * (M_PI / 180.0);
    float al = arm.alpha * (M_PI / 180.0);

    // wiersz 1
    result[0][0] = cos(th);
    result[0][1] = -sin(th) * cos(al);
    result[0][2] = sin(th) * sin(al);
    result[0][3] = arm.a * cos(th);

    // wiersz 2
    result[1][0] = sin(th);
    result[1][1] = cos(th) * cos(al);
    result[1][2] = -cos(th) * sin(al);
    result[1][3] = arm.a * sin(th);

    // wiersz 3
    result[2][0] = 0;
    result[2][1] = sin(al);
    result[2][2] = cos(al);
    result[2][3] = arm.d;

    // wiersz 4 (wektor skali/jednorodnosci)
    result[3][0] = 0;
    result[3][1] = 0;
    result[3][2] = 0;
    result[3][3] = 1;
}

void init_robot_kinematics()
{
    //  obrot serwa bazy (theta) oraz odleglosc miedzy frame 0 a serwo 2
    robot_joints[0].d = L1_BASE_HEIGHT;
    robot_joints[0].a = 0.0f;
    robot_joints[0].alpha = 90.0f;
    robot_joints[0].theta = 0.0f;

    // obrot serwa 2 oraz odleglosc miedzy 2 a 3
    robot_joints[1].d = 0.0f;
    robot_joints[1].a = L2_ARM1_LENGTH;
    robot_joints[1].alpha = 0.0f;
    robot_joints[1].theta = 0.0f;

    // obrot serwa 3 oraz odleglosc miedzy 3 a 4
    robot_joints[2].d = 0.0f;
    robot_joints[2].a = L3_ARM2_LENGTH;
    robot_joints[2].alpha = 0.0f;
    robot_joints[2].theta = 0.0f;

    //  obrot serwa 4 oraz odleglosc miedzy 4 a chwytak
    robot_joints[3].d = 0.0f;
    robot_joints[3].a = L4_GRIPPER_OFFSET;
    robot_joints[3].alpha = 0.0f; // tego nie zmieniac bo my mamy osie rownolegle
    robot_joints[3].theta = 0.0f;
}

// xyz na podstawie terazniejszych katow
void calculate_forward_kinematics(float final_matrix[4][4])
{
    // 0 0 0
    float current_matrix[4][4] = {
        {1, 0, 0, 0},
        {0, 1, 0, 0},
        {0, 0, 1, 0},
        {0, 0, 0, 1}};

    float temp_matrix[4][4];
    float result_matrix[4][4];

    // idziemy przez 4 serwa
    for (int i = 0; i < 4; i++)
    {
        // dh dla przegubu teraz
        generate_dh_matrix(robot_joints[i], temp_matrix);

        // curr * nowy dh
        multiply_matrices(current_matrix, temp_matrix, result_matrix);

        // kopiujemy wynik
        for (int row = 0; row < 4; row++)
        {
            for (int col = 0; col < 4; col++)
            {
                current_matrix[row][col] = result_matrix[row][col];
            }
        }
    }

    // po zakonczeniu mamy ostateczny matrix z wszystkimi pozycjami robota
    // macierz tranformacji + wektor przeniesienia = final_matrix
    for (int row = 0; row < 4; row++)
    {
        for (int col = 0; col < 4; col++)
        {
            final_matrix[row][col] = current_matrix[row][col];
        }
    }
}

// SEKCJA ESP ///////////////////////////////////////////////////////////////////////////////////////////////////

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
    .clk_source = I2C_CLK_SRC_DEFAULT,    // zegar systemowy
    .i2c_port = -1,                       // -1 oznacza, że ma być użyty domyślny port I2C
    .scl_io_num = I2C_MASTER_SCL_IO,      // pin 22
    .sda_io_num = I2C_MASTER_SDA_IO,      // pin 21
    .glitch_ignore_cnt = 7,               // filtorwanie zaklocen, standardowa wartosc dla esp32
    .flags.enable_internal_pullup = true, // rezystory pull up na pinach scl i sda
}; // konfiguracja master busa

// konfiguracja urzadzenia (slave)
i2c_device_config_t dev_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,       // 7 bitowy adres dla modulu pca9865
    .device_address = I2C_MASTER_DEVICE_ADDRESS, // domyslny adres dla modulu pca9865
    .scl_speed_hz = I2C_MASTER_FREQ_HZ,          // szybkosc i2c, standardowa wartosc dla esp32
};

i2c_master_bus_handle_t bus_handle; // uchwyt magistrali I2c
i2c_master_dev_handle_t dev_handle; // uchwyt urzadzenia dla modulu pca9865

void i2c_init(void)
{
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));
    ESP_LOGI(TAG, "I2C initialized");
}

// modul pca9685 ma rozdzielczosc 12 bitowa czyli 4095 pozycji
// pelny cykl to 50 hz czyli 20 ms
// pozycja srodkowa dla ser analogowych wymaga podania stanu wysokiego przez
// dokladnie 1.5 ms
// czyli pozadana liczba krokow to 1.5 ms / 20 ms * 4095 = 307
void reset_servos_position(void)
{
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
    buf[0] = 0xFE;
    buf[1] = 0x79;
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, buf, 2, -1));
    vTaskDelay(pdMS_TO_TICKS(10));
    // wlaczamy auto-inkrementacja
    buf[0] = 0x00;
    buf[1] = 0x20;
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, buf, 2, -1));
    vTaskDelay(pdMS_TO_TICKS(10));
    // ustawiamy pozycji srodka (resetu) dla wszystkich serw naraz
    uint16_t off_val = 307;
    uint8_t pwn_data[5];

    pwn_data[0] = 0xFA;           // adres poczatkowy
    pwn_data[1] = 0;              // ON_L
    pwn_data[2] = 0;              // ON_H
    pwn_data[3] = off_val & 0xFF; // OFF_L wyciagamy dolne 8 bitow
    pwn_data[4] = off_val >> 8;   // OFF_H wyciagamy gorne 8 bitow
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

    i2c_init();              // odpalamy master bus magistrale I2C
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
