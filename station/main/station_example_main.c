/* WiFi station Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "app_mqtt.h"
#include <stdio.h>

#include <stdlib.h>
#include <inttypes.h>
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#define EXAMPLE_ESP_WIFI_SSID      "Turtlee"
#define EXAMPLE_ESP_WIFI_PASS      "11111111"
#define EXAMPLE_ESP_MAXIMUM_RETRY  5

#if CONFIG_ESP_WPA3_SAE_PWE_HUNT_AND_PECK
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HUNT_AND_PECK
#define EXAMPLE_H2E_IDENTIFIER ""
#elif CONFIG_ESP_WPA3_SAE_PWE_HASH_TO_ELEMENT
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HASH_TO_ELEMENT
#define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#elif CONFIG_ESP_WPA3_SAE_PWE_BOTH
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_BOTH
#define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#endif
#if CONFIG_ESP_WIFI_AUTH_OPEN
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN
#elif CONFIG_ESP_WIFI_AUTH_WEP
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WEP
#elif CONFIG_ESP_WIFI_AUTH_WPA_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WAPI_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WAPI_PSK
#endif
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

#define TOUCH_GPIO1 7
#define TOUCH_GPIO2 6
#define TOUCH_GPIO3 5
#define TOUCH_GPIO4 4
#define GPIO_POWER_CONTROL 8
#define GPIO_INPUT_PIN_SEL ((1ULL << TOUCH_GPIO1) | (1ULL << TOUCH_GPIO2) | (1ULL << TOUCH_GPIO3) | (1ULL << TOUCH_GPIO4))
#define OUT_GPIO1 7
#define OUT_GPIO2 6
#define OUT_GPIO3 5
#define OUT_GPIO4 4
#define GPIO_OUTPUT_IO_0    3
#define GPIO_OUTPUT_IO_1    2
#define GPIO_OUTPUT_PIN_SEL ((1ULL << OUT_GPIO1) | (1ULL << OUT_GPIO2) | (1ULL << OUT_GPIO3) | (1ULL << OUT_GPIO4)| (1ULL << GPIO_OUTPUT_IO_0)| (1ULL << GPIO_OUTPUT_IO_1))
#define ESP_INTR_FLAG_DEFAULT 0
#define LED_COUNT           8
#define START_FRAME         0x00000000
#define END_FRAME           0xFFFFFFFF
#define CLOCK_DELAY_US      200
#define COLOR_COUNT (sizeof(led_colors) / sizeof(led_colors[0]))

// Tag cho ESP_LOG
#define TAG "GPIO Touch Sensor"
typedef struct {
    uint8_t brightness; 
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} led_t;
led_t leds[LED_COUNT] = {
    {31, 255, 0, 0},
    {31, 255, 0, 0}, 
    {31, 255, 0, 0},
    {31, 255, 0, 0}, 
    {31, 255, 0, 0}, 
    {31, 255, 0, 0},
    {31, 255, 0, 0}
};
led_t led_colors[] = {
    {31, 255, 0, 0},    // Đỏ
    {31, 0, 255, 0},    // Xanh lá
    {31, 0, 0, 255},    // Xanh dương
    {31, 255, 255, 0},  // Vàng
    {31, 0, 255, 255},  // Lục lam
    {31, 255, 0, 255},  // Tím
    {31, 255, 255, 255},// Trắng
    {31, 128, 128, 128} // Xám
};
uint32_t create_led_data(led_t led) {
    return ((0b111 << 29) | 
            ((led.brightness & 0x1F) << 24) | 
            ((led.blue & 0xFF) << 16) | 
            ((led.green & 0xFF) << 8) | 
            (led.red & 0xFF));
}

void send_data(uint32_t data) {
    for (int i = 31; i >= 0; i--) {
        int bit = (data >> i) & 1;
        gpio_set_level(GPIO_OUTPUT_IO_0, bit);
        gpio_set_level(GPIO_OUTPUT_IO_1, 1);
        esp_rom_delay_us(CLOCK_DELAY_US);
        gpio_set_level(GPIO_OUTPUT_IO_1, 0);
        esp_rom_delay_us(CLOCK_DELAY_US);
    }
}

void send_leds_data() {
    send_data(START_FRAME); 
    for (int i = 0; i < LED_COUNT; i++) {
        send_data(create_led_data(leds[i]));
    }
    send_data(END_FRAME);
    gpio_set_level(GPIO_OUTPUT_IO_0, 0);
}
void update_led_colors(int color_index) {
    for (int i = 0; i < LED_COUNT; i++) {
        leds[i] = led_colors[color_index];
    }
}
void turn_off_leds() {
    for (int i = 0; i < LED_COUNT; i++) {
        leds[i] = (led_t){0, 0, 0, 0};
    }
}
void shift_led_colors() {
    led_t temp = leds[LED_COUNT - 1]; 
    for (int i = LED_COUNT - 1; i > 0; i--) {
        leds[i] = leds[i - 1];
    }
    leds[0] = temp;
}
void blink_leds(int led1_idx, int led2_idx)
{
    static int state = 0;
    if (state == 0)
    {
        leds[led1_idx].brightness = 0;
        leds[led2_idx].brightness = 0;
    }
    else
    {
        leds[led1_idx].brightness = 31;
        leds[led2_idx].brightness = 31;
    }
    state = !state;
}
void gradient_effect() {
    static uint8_t brightness = 0;
    static int step = 1; 

    for (int i = 0; i < LED_COUNT; i++) {
        leds[i].brightness = brightness;
    }
    brightness += step;
    if (brightness == 0 || brightness == 31) {
        step = -step;
    }
}

void control_led_cycle() {
    static int color_index = 0;
    turn_off_leds();
    send_leds_data();
    vTaskDelay(pdMS_TO_TICKS(500));
    update_led_colors(color_index);
    send_leds_data();
    vTaskDelay(pdMS_TO_TICKS(1000));
    color_index = (color_index + 1) % COLOR_COUNT;
}
void wave_effect() {
    static int position = 0; 
    static int direction = 1;
    static uint8_t color_index = 0;
    uint8_t colors[6][3] = {
        {255, 0, 0},
        {255, 127, 0},
        {255, 255, 0},
        {0, 255, 0},
        {0, 0, 255},
        {75, 0, 130}
    };
    for (int i = 0; i < LED_COUNT; i++) {
        if (i == position) {
            leds[i].brightness = 31;
        } else if (i == (LED_COUNT - position - 1)) {
            leds[i].brightness = 31;
        } else {
            leds[i].brightness = 0;
        }
        leds[i].red = colors[color_index][0];
        leds[i].green = colors[color_index][1];
        leds[i].blue = colors[color_index][2];
    }
    position += direction;
    if (position == LED_COUNT - 1 || position == 0) {
        direction = -direction;
        color_index = (color_index + 1) % 6;
    }
}

static int s_retry_num = 0;


static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,.threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
            .sae_pwe_h2e = ESP_WIFI_SAE_MODE,
            .sae_h2e_identifier = EXAMPLE_H2E_IDENTIFIER,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
        mqtt_app_start();
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}
#define LEDS_PER_SENSOR 2
static bool sensor_status[] = {false, false, false, false};
static int sensor_idx = -1;
gpio_num_t output_pin=-1;
static QueueHandle_t gpio_evt_queue = NULL;
static void IRAM_ATTR gpio_isr_handler(void *arg)
{
   uint32_t gpio_num = (uint32_t)arg;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (gpio_evt_queue != NULL) {
        xQueueSendFromISR(gpio_evt_queue, &gpio_num, &xHigherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void update_led_color_based_on_touch(int sensor_idx, bool status) {
    int led_start = sensor_idx * LEDS_PER_SENSOR; 
    for (int i = 0; i < LEDS_PER_SENSOR; i++) {
        int led_idx = led_start + i; 
        if (status) { 
            leds[led_idx].brightness = 31;
        } else { 
            leds[led_idx].brightness = 7;
        }
    }
    printf("LEDs for Sensor %d updated: %s\n", sensor_idx, status ? "ON" : "OFF");
}
void special_mode(int idx, uint32_t gpio_num)
{
    ESP_LOGI(TAG, "Special Mode Activated for Sensor[%d], GPIO[%" PRIu32 "]", idx, gpio_num);
    int led1_idx = idx * 2;
    int led2_idx = led1_idx + 1;
    led_t original_led_1 = leds[led1_idx];
    led_t original_led_2 = leds[led2_idx];
    for (int i = 0; i < 3; i++)
    {
        blink_leds(led1_idx, led2_idx);
        vTaskDelay(pdMS_TO_TICKS(100)); 
    }
    leds[led1_idx] = original_led_1;
    leds[led2_idx] = original_led_2;
}
static void gpio_task_example(void *arg)
{
    uint32_t io_num;
    TickType_t press_start_time = 0;
    TickType_t press_duration = 0;
    const TickType_t LONG_PRESS_THRESHOLD = 2000 / portTICK_PERIOD_MS;
    for (;;)
    {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY))
        {
            if (gpio_get_level(io_num) == 1){
                press_start_time = xTaskGetTickCount();
                ESP_LOGI(TAG, "Button on GPIO[%" PRIu32 "] Pressed", io_num);
            }
            else{
                press_duration = xTaskGetTickCount() - press_start_time;

                if (press_duration >= LONG_PRESS_THRESHOLD)
                {
                    ESP_LOGI(TAG, "GPIO[%" PRIu32 "] Long Press Detected!", io_num);
                    switch (io_num)
                    {
                    case TOUCH_GPIO1:
                        sensor_idx = 0;
                        output_pin = OUT_GPIO1;
                        break;
                    case TOUCH_GPIO2:
                        sensor_idx = 1;
                        output_pin = OUT_GPIO2;
                        break;
                    case TOUCH_GPIO3:
                        sensor_idx = 2;
                        output_pin = OUT_GPIO3;
                        break;
                    case TOUCH_GPIO4:
                        sensor_idx = 3;
                        output_pin = OUT_GPIO4;
                        break;
                    default:
                        break;
                    }

                    if (sensor_idx != -1 && output_pin != -1)
                    {
                        ESP_LOGI(TAG, "GPIO[%" PRIu32 "] Long Press -> Special Mode Activated!", io_num);
                        special_mode(sensor_idx, io_num);
                    }
                }
                else
                {
                    ESP_LOGI(TAG, "GPIO[%" PRIu32 "] Short Press Detected!", io_num);
                    switch (io_num)
                    {
                    case TOUCH_GPIO1:
                        sensor_idx = 0;
                        output_pin = OUT_GPIO1;
                        break;
                    case TOUCH_GPIO2:
                        sensor_idx = 1;
                        output_pin = OUT_GPIO2;
                        break;
                    case TOUCH_GPIO3:
                        sensor_idx = 2;
                        output_pin = OUT_GPIO3;
                        break;
                    case TOUCH_GPIO4:
                        sensor_idx = 3;
                        output_pin = OUT_GPIO4;
                        break;
                    default:
                        break;
                    }
                    if (sensor_idx != -1 && output_pin != -1)
                    {
                        sensor_status[sensor_idx] = !sensor_status[sensor_idx];
                        gpio_set_level(output_pin, sensor_status[sensor_idx]);
                        ESP_LOGI(TAG, "GPIO[%" PRIu32 "] -> Output[%d]: %s",
                                 io_num, output_pin, sensor_status[sensor_idx] ? "ON" : "OFF");
                        publish_state_to_mqtt(sensor_status[sensor_idx], io_num);
                        update_led_color_based_on_touch(sensor_idx,sensor_status[sensor_idx]);
                    }
                }
            }
        }
    }
}
void touch_gpio_init()
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    io_conf.intr_type = GPIO_INTR_ANYEDGE; 
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 0;   
    io_conf.pull_down_en = 1;
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    gpio_config(&io_conf);
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(TOUCH_GPIO1, gpio_isr_handler, (void *)TOUCH_GPIO1);
    gpio_isr_handler_add(TOUCH_GPIO2, gpio_isr_handler, (void *)TOUCH_GPIO2);
    gpio_isr_handler_add(TOUCH_GPIO3, gpio_isr_handler, (void *)TOUCH_GPIO3);
    gpio_isr_handler_add(TOUCH_GPIO4, gpio_isr_handler, (void *)TOUCH_GPIO4);
    gpio_set_level(OUT_GPIO1, 0);
    gpio_set_level(OUT_GPIO2, 0);
    gpio_set_level(OUT_GPIO3, 0);
    gpio_set_level(OUT_GPIO4, 0);
}
void control_power_gpio_init()
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pin_bit_mask = (1ULL << GPIO_POWER_CONTROL);
    gpio_config(&io_conf);
}
void turn_on_touch_sensor()
{
    gpio_set_level(GPIO_POWER_CONTROL, 1); // Đặt IO8 lên mức cao
    ESP_LOGI("POWER_CONTROL", "Touch sensor activated");
}
void turn_off_touch_sensor()
{
    gpio_set_level(GPIO_POWER_CONTROL, 0); // Đặt IO8 về mức thấp
    ESP_LOGI("POWER_CONTROL", "Touch sensor deactivated");
}
void update_led_color(const char *method, uint8_t value) {
    for (int i = 0; i < LED_COUNT; i++) {
        if (strcmp(method, "red") == 0) {
            leds[i].red = value;
            ESP_LOGI("LED_UPDATE", "Updated LED[%d] Red to %d", i, value);
        } else if (strcmp(method, "green") == 0) {
            leds[i].green = value;
            ESP_LOGI("LED_UPDATE", "Updated LED[%d] Green to %d", i, value);
        } else if (strcmp(method, "blue") == 0) {
            leds[i].blue = value;
            ESP_LOGI("LED_UPDATE", "Updated LED[%d] Blue to %d", i, value);
        } else if (strcmp(method, "brightness") == 0) {
            leds[i].brightness = value;
            ESP_LOGI("LED_UPDATE", "Updated LED[%d] Brightness to %d", i, value);
        } else {
            ESP_LOGW("LED_UPDATE", "Unsupported method: %s", method);
        }
    }
}
void gradient_task(void *param) {
    while (1) {
        gradient_effect();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void mqtt_data_callback(uint8_t *data, uint16_t length) {
    char *pt = (char *)data;
    uint32_t gpio_num = 0;
    uint8_t value = 0;

    static TaskHandle_t gradient_task_handle = NULL;
    char *start = strstr(pt, "\"method\":");
    if (start != NULL) {
        start += 9;
        while (*start == ' ' || *start == '"') start++;
        char method[32] = {0};
        char *end = strchr(start, '"');
        if (end != NULL) {
            size_t len = end - start;
            if (len < sizeof(method)) {
                strncpy(method, start, len);
                method[len] = '\0';
                ESP_LOGI(TAG, "Received method: %s", method);
                if (strcmp(method, "rainbow") == 0) {
                    if (gradient_task_handle == NULL) {
                        ESP_LOGI(TAG, "Starting gradient effect.");
                        xTaskCreate(gradient_task, "GradientTask", 2048, NULL, 5, &gradient_task_handle);
                    } else {
                        ESP_LOGI(TAG, "Stopping gradient effect.");
                        vTaskDelete(gradient_task_handle);
                        gradient_task_handle = NULL;
                    }
                    return;
                }
                if (gradient_task_handle != NULL) {
                    ESP_LOGI(TAG, "Stopping gradient effect due to other action.");
                    vTaskDelete(gradient_task_handle);
                    gradient_task_handle = NULL;
                }
                if (strncmp(method, "GPIO", 4) == 0) {
                    gpio_num = (uint32_t)strtol(method + 4, NULL, 10);
                    if (gpio_num >= 4 && gpio_num <= 7) {
                        char *params_start = strstr(pt, "\"params\":");
                        if (params_start != NULL) {
                            params_start += 9;
                            while (*params_start == ' ' || *params_start == ':') params_start++;
                            if (strncmp(params_start, "true", 4) == 0) {
                                value = 1;
                            } else if (strncmp(params_start, "false", 5) == 0) {
                                value = 0;
                            } else {
                                ESP_LOGE(TAG, "Invalid params value for GPIO.");
                                return;
                            }
                            ESP_LOGI(TAG, "GPIO[%s] -> Params: %d", method, value);
                        } else {
                            ESP_LOGE(TAG, "Params not found for GPIO method.");
                            return;
                        }
                        int sensor_idx = -1;
                        int output_pin = -1;

                        switch (gpio_num) {
                            case TOUCH_GPIO4:
                                sensor_idx = 3;
                                output_pin = OUT_GPIO4;
                                break;
                            case TOUCH_GPIO3:
                                sensor_idx = 2;
                                output_pin = OUT_GPIO3;
                                break;
                            case TOUCH_GPIO2:
                                sensor_idx = 1;
                                output_pin = OUT_GPIO2;
                                break;
                            case TOUCH_GPIO1:
                                sensor_idx = 0;
                                output_pin = OUT_GPIO1;
                                break;
                            default:
                                ESP_LOGE(TAG, "Unsupported GPIO number: %" PRIu32, gpio_num);
                                return;
                        }
                        if (sensor_idx != -1 && output_pin != -1) {
                            sensor_status[sensor_idx] = value;
                            gpio_set_level(output_pin, value);
                            ESP_LOGI(TAG, "GPIO[%s] -> Output[%d]: %s",
                                     method, output_pin, value ? "ON" : "OFF");
                            update_led_color_based_on_touch(sensor_idx, sensor_status[sensor_idx]);
                        }
                    } else {
                        ESP_LOGE(TAG, "Invalid GPIO number: %" PRIu32, gpio_num);
                        return;
                    }
                } else {
                    char *params_start = strstr(pt, "\"params\":");
                    if (params_start != NULL) {
                        params_start += 9;
                        while (*params_start == ' ' || *params_start == ':') params_start++;
                        value = (uint8_t)strtol(params_start, NULL, 10);
                        ESP_LOGI(TAG, "Color[%s] -> Params: %d", method, value);
                        if (strcmp(method, "red") == 0 || strcmp(method, "green") == 0 || strcmp(method, "blue") == 0 || strcmp(method, "brightness") == 0) {
                            update_led_color(method, value);
                        } else {
                            ESP_LOGW("MQTT_CALLBACK", "Method %s is not related to LED color.", method);
                        }
                    } else {
                        ESP_LOGE(TAG, "Params not found for color method.");
                        return;
                    }
                }
            } else {
                ESP_LOGE(TAG, "Method name is too long.");
                return;
            }
        } else {
            ESP_LOGE(TAG, "Invalid method format in message.");
            return;
        }
    } else {
        ESP_LOGE(TAG, "Method not found in message.");
        return;
    }
}


void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();
    ESP_LOGI(TAG, "Initializing touch sensors using GPIO");

    touch_gpio_init();
    control_power_gpio_init();
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);
    mqtt_data_pt_set_callback(mqtt_data_callback); 
    while (1) {
         send_leds_data();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

