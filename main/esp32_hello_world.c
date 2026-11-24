#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include <math.h>
#include <stdbool.h>
#include <string.h>
#include "esp_vfs_dev.h"
#include "sdkconfig.h"

// TUTORIAL DLA TESTERA
// 1. najpierw musisz zmierzyc i ustawic dobra wartosci fizycznych wymiarow robota
// w sekcji global defines physical w mm ( wiec w set pozniej tez podawaj mm)
// 2. narazie dziala tylko sterowanie w terminale za pomoca "GET" i "SET <x> <y> <z>" oczywiscie bez tych <>
// 3. jezeli serwa ida na odwrot niz sie wydaje lub trzeszca i brzmi jakby mialo wybuchnac to trzeba zmienic znak i odjac 90.0f stopni w move_your_robot
// 4. w console task zmienic false na true w move_your_robot jak chce sie tez sterowac serwem chwytaka
// 5. wpisz "MOVE <numer_serwa>" np. "MOVE 1" by przesunac wybrane serwo (0, 1, 2 lub 3) o 10 stopni w celach testowych.

// GLOBAL DEFINES ESP

#define I2C_MASTER_SCL_IO 22           // Pin zegara (SCL)
#define I2C_MASTER_SDA_IO 21           // Pin danych (SDA)
#define I2C_MASTER_FREQ_HZ 100000      //  100 kHz
#define I2C_MASTER_DEVICE_ADDRESS 0x40 // domyslny adres dla modulu pca9865
#define BLINK_GPIO 2

static const char *TAG = "hello_world";

// GLOBAL DEFINES PHYSICAL

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

void set_servo_angle(int servo_number, float angle);
//  struct do trzymania informacji pojedynczego serwa/ramienia

typedef struct
{
    float theta;
    float d;
    float a;
    float alpha;
} SingleArm;

SingleArm robot_joints[4]; // pelne 4 osie

// Zmienna kierunkowa dla trybu testowego MOVE
bool test_move_direction_up = true;

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

u_int16_t angle_to_pwm(float angle)
{
    if (angle > 180.0f)
        angle = 180.0f;
    if (angle < 0.0f)
        angle = 0.0f;
    // to naklada jeden zakres na drugi
    // funkcja wzieta z neta (chyba z excela )
    float pwm_float = 102.0f + (angle / 180.0f) * (512.0f - 102.0f);

    return (u_int16_t)pwm_float;
}
void set_servo_angle(int servo_number, float angle)
{
    u_int16_t pwm = angle_to_pwm(angle);
    u_int8_t starting_adress = 0x06 + (4 * servo_number);
    u_int8_t pwm_data[5];
    pwm_data[0] = starting_adress; // adres poczatkowy
    pwm_data[1] = 0;               // ON_L
    pwm_data[2] = 0;               // ON_H
    pwm_data[3] = pwm & 0xFF;      // OFF_L wyciagamy dolne 8 bitow
    pwm_data[4] = pwm >> 8;        // OFF_H wyciagamy gorne 8 bitow
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, pwm_data, 5, -1));
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_LOGI(TAG, "SERVO ANGLE CHANGED");
}

// inverse kinematics - podajemy wspolrzedne w base 0 i z tego obliczamy thety dla serw zeby osiagnac
// ta pozycje
bool calculate_inverse_kinematics(float target_x, float target_y, float target_z,
                                  bool active_gripper, float target_pitch_deg,
                                  float *out_t0, float *out_t1, float *out_t2, float *out_t3)
{
    // oliczenie kata bazy (theta 0)
    float theta0 = atan2(target_y, target_x);

    // Całkowity rzut poziomy na płaszczyznę blatu (odległość do celu)
    float r = sqrt(target_x * target_x + target_y * target_y);

    // tworzymy trojkat 2d od drugiego serwa
    // wyrzucamy serwo 1 dla stworzenia problemu 2d
    // Zredukowana wysokość (od osi barku, a nie od stołu)
    float z = target_z - L1_BASE_HEIGHT;

    // Zmienne robocze dla geometrii trójkąta (do czego ma sięgnąć łokieć)
    float target_r, target_z_eff;
    float L_last; // Długość ostatniego członu użytego do obliczeń trójkąta

    // budowanie trojkata w zaleznosci od ilosci osi
    if (active_gripper)
    {
        // metoda srodka nadgarstka (4 osie)
        float pitch_rad = target_pitch_deg * (M_PI / 180.0f);

        // Cofamy się od punktu docelowego o długość chwytaka (L4) pod zadanym kątem
        target_r = r - (L4_GRIPPER_OFFSET * cos(pitch_rad));
        target_z_eff = z - (L4_GRIPPER_OFFSET * sin(pitch_rad));

        // Trójkąt budujemy używając tylko długości ramienia (L2) i przedramienia (L3)
        L_last = L3_ARM2_LENGTH;
    }
    else
    {
        // sztywny chwytak 3 osie
        target_r = r;
        target_z_eff = z;

        // Trójkąt budujemy używając sztywnego pręta (L3 + L4)
        L_last = L3_ARM2_LENGTH + L4_GRIPPER_OFFSET;
    }

    // klasyczna kinematyka
    float L1 = L2_ARM1_LENGTH; // Główna kość ramienia

    // Odległość w prostej linii od barku do obliczonego celu (nadgarstka lub końca chwytaka)
    float d = sqrt(target_r * target_r + target_z_eff * target_z_eff);

    // Zabezpieczenie fizyczne (czy cel nie jest za daleko lub za blisko)
    if (d > (L1 + L_last) || d < fabs(L1 - L_last))
    {
        ESP_LOGE(TAG, "Cel poza fizycznym zasięgiem ramienia!");
        return false;
    }
    // tutaj sie robi troche ciezko z zrozumieniem tego algorytmu
    // Twierdzenie cosinusów dla łokcia (theta 2)
    float cos_theta2 = (target_r * target_r + target_z_eff * target_z_eff - L1 * L1 - L_last * L_last) / (2 * L1 * L_last);
    if (cos_theta2 > 1.0f)
        cos_theta2 = 1.0f;
    if (cos_theta2 < -1.0f)
        cos_theta2 = -1.0f;

    // Konfiguracja "elbow up" (wypukły w górę)
    float theta2 = -acos(cos_theta2);

    // Geometria dla barku (theta 1)
    float gamma = atan2(target_z_eff, target_r);
    float alpha = atan2(L_last * sin(theta2), L1 + L_last * cos(theta2));
    float theta1 = gamma - alpha;

    // wyliczenie katow theta
    float theta3 = 0.0f;
    if (active_gripper)
    {
        float pitch_rad = target_pitch_deg * (M_PI / 180.0f);
        // Ponieważ kąt pitch to całkowity kąt chwytaka względem podłoża,
        // a thety dodają się do siebie w łańcuchu kinematycznym:
        // Pitch = theta1 + theta2 + theta3  =>  theta3 = Pitch - theta1 - theta2
        theta3 = pitch_rad - (theta1 + theta2);
    }

    // Zapisujemy wyliczone wartości (zamienione na stopnie) pod wskaźniki
    *out_t0 = theta0 * (180.0f / M_PI);
    *out_t1 = theta1 * (180.0f / M_PI);
    *out_t2 = theta2 * (180.0f / M_PI);
    *out_t3 = theta3 * (180.0f / M_PI);

    return true;
}

void move_robot_to_xyz(float x, float y, float z, bool use_gripper, float pitch)
{
    float t0, t1, t2, t3;

    // Wołamy naszą nową funkcję IK
    if (calculate_inverse_kinematics(x, y, z, use_gripper, pitch, &t0, &t1, &t2, &t3))
    {

        // Aktualizujemy globalną tablicę (żeby funkcja Forward Kinematics miała dobre dane)
        robot_joints[0].theta = t0;
        robot_joints[1].theta = t1;
        robot_joints[2].theta = t2;
        robot_joints[3].theta = t3; // Nowe 4. serwo

        // Aplikujemy offsety mechaniczne.
        // Jeśli serwo środkuje na 90 stopniach, dodajemy matematyczny kąt do fizycznego środka.
        // tutaj dajemy plusa bo zakladam ze serwo jak idzie w dodatnia strone to idzie do gory ramie
        // jest to dodanie fizycznosci do czystej matmy
        set_servo_angle(0, 90.0f + t0);
        set_servo_angle(1, 90.0f + t1); // Bark
        set_servo_angle(2, 90.0f + t2); // Łokieć

        if (use_gripper)
        {
            set_servo_angle(3, 90.0f + t3); // Nadgarstek
        }
        else
        {
            // Sztywny chwytak - trzymaj ustaloną pozycję, by się nie majtał (np. równo ze środkiem)
            set_servo_angle(3, 90.0f);
        }

        ESP_LOGI(TAG, "Wykonano ruch do cel: %.1f, %.1f, %.1f (Pitch: %.1f)", x, y, z, pitch);
    }
}

// funckja testowa do poruszania serwem o 10 stopni
void test_move_servo(int servo_num)
{
    if (servo_num < 0 || servo_num > 3)
    {
        ESP_LOGE(TAG, "nieprawidłowy numer serwa! uzyj 0-3.");
        return;
    }

    // pobieramy aktualny matematyczny kat z pamieci
    float current_theta = robot_joints[servo_num].theta;

    // zmieniamy kierunek jak dojedziemy do limitu
    if (current_theta >= 70.0f)
        test_move_direction_up = false;
    if (current_theta <= -70.0f)
        test_move_direction_up = true;

    // przesuwamy o 10 stopni w wybranym kierunku
    if (test_move_direction_up)
    {
        current_theta += 10.0f;
    }
    else
    {
        current_theta -= 10.0f;
    }

    // zapisujemy katu do pamieci
    robot_joints[servo_num].theta = current_theta;

    // aplikujemy offset 90 stopni i wysylamy
    // WAZNE TUTAJ TRZEBA DODAC SWITCH DLA servo_num JEZELI KTORES Z SERW PRZY KALIBRACJI
    // WYJDZIE ZLE!!!!!!!
    set_servo_angle(servo_num, 90.0f + current_theta);

    ESP_LOGI(TAG, "TEST MOVE: serwo %d przesuniete. nowy kat mat.: %.1f, kat fiz.: %.1f",
             servo_num, current_theta, 90.0f + current_theta);
}

// SEKCJA RTOS ///////////////////////////////////////////////////////////////////////////////////////////////////
void console_task(void *pvParameters)
{
    char line[128]; // bufor na info z terminalu
    ESP_LOGI(TAG, "terminal gotowy. Wpisz komende (np. SET 100 0 50, GET, lub MOVE 1):");
    while (1)
    {
        if (fgets(line, sizeof(line), stdin) != NULL)
        {
            if (strlen(line) <= 2)
            {
                continue;
            }
            if (strncmp(line, "GET", 3) == 0)
            {
                float f_matrix[4][4];
                calculate_forward_kinematics(f_matrix);

                // wyciagamy tylko wektor translacji p
                ESP_LOGI(TAG, "Aktualna pozycja: X=%.1f, Y=%.1f, Z=%.1f",
                         f_matrix[0][3], f_matrix[1][3], f_matrix[2][3]);
            }
            else if (strncmp(line, "SET", 3) == 0)
            {
                float x, y, z;
                // sciagamy xyz
                int wynik = sscanf(line, "SET %f %f %f", &x, &y, &z);
                if (wynik == 3)
                {
                    ESP_LOGI(TAG, "Ruch robota zaaktywowany");
                    move_robot_to_xyz(x, y, z, false, 0);
                }
                else
                {
                    ESP_LOGI(TAG, "Blad skladni SET! Skladnia SET: 'SET <x> <y> <z>' ");
                }
            }
            else if (strncmp(line, "MOVE", 4) == 0)
            {
                int servo_id;
                int wynik = sscanf(line, "MOVE %d", &servo_id);
                if (wynik == 1)
                {
                    ESP_LOGI(TAG, "Uruchamiam test serwa %d", servo_id);
                    test_move_servo(servo_id);
                }
                else
                {
                    ESP_LOGI(TAG, "Blad skladni MOVE! Skladnia MOVE: 'MOVE <numer_serwa>' (0-3)");
                }
            }
            else
            {
                ESP_LOGI(TAG, "Nieznana komenda! %s", line);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
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

void app_main(void)
{
    ESP_LOGI(TAG, "uruchamianie esp32 z esp-idf...");
    gpio_reset_pin(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

    i2c_init();              // odpalamy master bus magistrale I2C
    reset_servos_position(); // resetujemy pozycje serw
    init_robot_kinematics(); // przygotowujemy dane do obliczen

    // ujednoliecenie znakow w stdin dla komunikacji
    // to jest po to zeby esp nie robilo konfliktu z stdin dla c
    esp_vfs_dev_uart_port_set_rx_line_endings(CONFIG_ESP_CONSOLE_UART_NUM, ESP_LINE_ENDINGS_CR);
    esp_vfs_dev_uart_port_set_tx_line_endings(CONFIG_ESP_CONSOLE_UART_NUM, ESP_LINE_ENDINGS_CRLF);

    // odpalamy console task ktory od uzytkownika bierze komendy
    // dajemy mu troche pamieci stacku bo sscanf zzera pamiec
    xTaskCreate(console_task, "console_task", 4096, NULL, 5, NULL);
    while (1)
    {
        ESP_LOGI(TAG, "Hello world from ESP32!");
        gpio_set_level(BLINK_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(1000));
        gpio_set_level(BLINK_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}