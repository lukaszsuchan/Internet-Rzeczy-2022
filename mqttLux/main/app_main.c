/* MQTT (over TCP) Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_event.h"
#include "esp_netif.h"
#include <esp_err.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "lwip/err.h"
#include "lwip/sys.h"

#include "esp_log.h"
#include "mqtt_client.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <string.h>
#include "yl69.h"
#include "dht.h"
#include "driver/adc.h"
#include "sdkconfig.h"
#include "cJSON.h"
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "esp_sleep.h"
#include "driver/i2c.h"
#include "driver/adc.h"
#include "esp_bt.h"
#include "driver/gpio.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include <esp_http_client.h>
#include "driver/timer.h"

#if defined(CONFIG_EXAMPLE_I2C_ADDRESS_LO)
#define ADDR BH1750_ADDR_LO
#endif
#if defined(CONFIG_EXAMPLE_I2C_ADDRESS_HI)
#define ADDR BH1750_ADDR_HI
#endif

#if defined(CONFIG_EXAMPLE_TYPE_DHT11)
#define SENSOR_TYPE DHT_TYPE_DHT11
#endif
#if defined(CONFIG_EXAMPLE_TYPE_AM2301)
#define SENSOR_TYPE DHT_TYPE_AM2301
#endif
#if defined(CONFIG_EXAMPLE_TYPE_SI7021)
#define SENSOR_TYPE DHT_TYPE_SI7021
#endif

#ifndef APP_CPU_NUM
#define APP_CPU_NUM PRO_CPU_NUM
#endif

#define FIRMWARE_VERSION 0.1

#define BH1750_ADDRESS 0x23
#define BH1750_POWER_DOWN 0x00
#define BH1750_POWER_ON 0x01
#define BH1750_RESET 0x07
#define BH1750_CONTINUOUS_HIGH_RES_MODE 0x10
#define BH1750_CONTINUOUS_LOW_RES_MODE 0x11

#define I2C_MASTER_SCL_IO 21        /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 22        /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0    /*!< I2C port number for master dev */
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master do not need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master do not need buffer */
#define I2C_MASTER_FREQ_HZ 100000   /*!< I2C master clock frequency */
#define BH1750_SENSOR_ADDR 0x23     /*!< BH1750 sensor address */

#define BUTTON_GPIO 5
#define RESET_BUTTON_GPIO 18
#define WAKE_UP_BUTTON_GPIO 15
#define DELAY_TIME_MS 10000
#define LED_PIN 2

bool is_low = false;
bool reset = false;
bool not_establised = true;
bool wake_up = false;
uint32_t time_in_us = 180;
uint32_t time_to_measure = 15;

#define YL_69_ADC_CHANNEL ADC1_CHANNEL_0

volatile char *user_wifi_ssid = "ssid";
volatile char *user_wifi_pass = "password";

typedef struct
{
    char ssid[32];
    char pass[64];
} wifi_credentials_t;

wifi_credentials_t wifi_credentials;

// ble config
#define DEVICE_INFO_SERVICE_UUID 0x180A

#define MANUFACTURER_NAME_CHAR 0xFEF4

#define SSID_CHAR 0xDEAD
#define PSK_CHAR 0xDEAE
#define IP_SERVER 0xDEAF
#define EXTERNAL_TEMPERATURE 0xDEB0
#define EXTERNAL_HUMIDITY 0xDEB1

#define BLE_NAME "BLE-PLANT"

char *TAG2 = "BLE-CONNECTION";

uint8_t ble_addr_type;

volatile bool were_ssid_given = false;
volatile bool were_psk_given = false;

TaskHandle_t xHandle = NULL;

void led_blink_setup()
{
    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
}

void led_blink()
{
    while (1)
    {
        gpio_set_level(LED_PIN, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        gpio_set_level(LED_PIN, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        //     // ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        //     // vTaskDelete(xHandle);
    }
}

void led_blink_connected()
{
    while (1)
    {
        gpio_set_level(LED_PIN, 0);
        vTaskDelay(200 / portTICK_PERIOD_MS);
        gpio_set_level(LED_PIN, 1);
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
}

void led_blink_delete()
{
    for (int i = 0; i < 5; i++)
    {
        gpio_set_level(LED_PIN, 0);
        vTaskDelay(200 / portTICK_PERIOD_MS);
        gpio_set_level(LED_PIN, 1);
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
}

void led_blink_wake_up()
{
    for (int i = 0; i < 2; i++)
    {
        gpio_set_level(LED_PIN, 0);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        gpio_set_level(LED_PIN, 1);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

void led_blink_no_wifi()
{
    for (int i = 0; i < 10; i++)
    {
        gpio_set_level(LED_PIN, 0);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        gpio_set_level(LED_PIN, 1);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
    gpio_set_level(LED_PIN, 0);
}

esp_err_t bh1750_init(i2c_port_t i2c_num)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BH1750_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, BH1750_POWER_ON, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

void turn_off_bh1750()
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BH1750_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x00, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_OK)
    {
        printf("BH1750 power off sucessfull\n");
    }
    else
    {
        printf("BH1750 power off failed\n");
    }
}

void turn_on_bh1750()
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BH1750_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x01, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_OK)
    {
        printf("BH1750 power on sucessfull\n");
    }
    else
    {
        printf("BH1750 power on failed\n");
    }
}

esp_err_t bh1750_measure(i2c_port_t i2c_num, uint16_t *light_intensity, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BH1750_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK)
    {
        return ret;
    }
    vTaskDelay(200 / portTICK_PERIOD_MS);
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BH1750_ADDRESS << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, (uint8_t *)light_intensity, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, ((uint8_t *)light_intensity) + 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    *light_intensity = (*light_intensity << 8) | *((uint8_t *)light_intensity + 1);
    return ret;
}
// ble implementation

void ble_app_advertise(void);

// callback from characteristic 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff
static int device_write_ssid(uint8_t conn_handle, uint8_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    vTaskDelay(100);
    char *incoming_data = (char *)ctxt->om->om_data;
    printf("incoming message: %s\n", incoming_data);
    cJSON *payload = cJSON_Parse(incoming_data);
    cJSON *ssid = cJSON_GetObjectItem(payload, "S");
    // cJSON *psk = cJSON_GetObjectItem(payload, "PSK");
    user_wifi_ssid = ssid->valuestring;
    // user_wifi_pass = psk->valuestring;
    // user_wifi_ssid = incoming_data;
    printf("WiFi Credentials SSID: %s\n", user_wifi_ssid);
    were_ssid_given = true;
    return 0;
}
static int device_write_psk(uint8_t conn_handle, uint8_t attr_handle, struct ble_gatt_access_ctxt *ctxt_psk, void *arg)
{
    char *incoming_data_psk = (char *)ctxt_psk->om->om_data;
    printf("incoming message: %s\n", incoming_data_psk);
    cJSON *payload = cJSON_Parse(incoming_data_psk);
    // cJSON *ssid = cJSON_GetObjectItem(payload, "SSID");
    cJSON *psk = cJSON_GetObjectItem(payload, "P");
    // user_wifi_ssid = ssid->valuestring;
    user_wifi_pass = psk->valuestring;
    // user_wifi_pass = incoming_data_psk;
    printf("WiFi Credentials PSK: %s\n", user_wifi_pass);
    were_psk_given = true;
    return 0;
}
static int device_write_ip_server(uint8_t conn_handle, uint8_t attr_handle, struct ble_gatt_access_ctxt *ctxt_psk, void *arg)
{
    char *incoming_data_psk = (char *)ctxt_psk->om->om_data;
    printf("incoming message: %s\n", incoming_data_psk);
    cJSON *payload = cJSON_Parse(incoming_data_psk);
    // cJSON *ssid = cJSON_GetObjectItem(payload, "SSID");
    cJSON *psk = cJSON_GetObjectItem(payload, "P");
    // user_wifi_ssid = ssid->valuestring;
    user_wifi_pass = psk->valuestring;
    // user_wifi_pass = incoming_data_psk;
    printf("WiFi Credentials PSK: %s\n", user_wifi_pass);
    were_psk_given = true;
    return 0;
}
static int device_write_external_temperature(uint8_t conn_handle, uint8_t attr_handle, struct ble_gatt_access_ctxt *ctxt_temp, void *arg)
{
    char *incoming_data_temp = (char *)ctxt_temp->om->om_data;
    printf("%s", incoming_data_temp);
    were_psk_given = true;
    return 0;
}
static int device_write_external_humidity(uint8_t conn_handle, uint8_t attr_handle, struct ble_gatt_access_ctxt *ctxt_psk, void *arg)
{
    char *incoming_data_psk = (char *)ctxt_psk->om->om_data;
    printf("incoming message: %s\n", incoming_data_psk);
    were_psk_given = true;
    return 0;
}
static int device_info(uint16_t con_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    os_mbuf_append(ctxt->om, "Wifi network details that you want to connect", strlen("Wifi network details that you want to connect"));
    return 0;
}
static const struct ble_gatt_svc_def gatt_svcs[] = {
    {.type = BLE_GATT_SVC_TYPE_PRIMARY,
     .uuid = BLE_UUID16_DECLARE(DEVICE_INFO_SERVICE_UUID),
     .characteristics = (struct ble_gatt_chr_def[]){
         {.uuid = BLE_UUID16_DECLARE(MANUFACTURER_NAME_CHAR),
          .flags = BLE_GATT_CHR_F_READ,
          .access_cb = device_info},
         {.uuid = BLE_UUID16_DECLARE(SSID_CHAR),
          .flags = BLE_GATT_CHR_F_WRITE,
          .access_cb = device_write_ssid},
         {.uuid = BLE_UUID16_DECLARE(PSK_CHAR),
          .flags = BLE_GATT_CHR_F_WRITE,
          .access_cb = device_write_psk},
          {.uuid = BLE_UUID16_DECLARE(IP_SERVER),
          .flags = BLE_GATT_CHR_F_WRITE,
          .access_cb = device_write_ip_server},
          {.uuid = BLE_UUID16_DECLARE(EXTERNAL_TEMPERATURE),
          .flags = BLE_GATT_CHR_F_WRITE,
          .access_cb = device_write_external_temperature},
          {.uuid = BLE_UUID16_DECLARE(EXTERNAL_HUMIDITY),
          .flags = BLE_GATT_CHR_F_WRITE,
          .access_cb = device_write_external_humidity},
         {0}}},
    {0}};

static int ble_gap_event(struct ble_gap_event *event, void *arg)
{
    vTaskDelay(1);
    switch (event->type)
    {
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI("GAP", "BLE GAP EVENT CONNECT %s", event->connect.status == 0 ? "OK!" : "FAILED!");
        if (event->connect.status != 0)
        {
            // start advertising again!
            ble_app_advertise();
        }
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI("GAP", "BLE GAP EVENT");
        ble_app_advertise();
        break;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI("GAP", "BLE GAP EVENT");
        ble_app_advertise();
        break;
    case BLE_GAP_EVENT_SUBSCRIBE:
        ESP_LOGI("GAP", "BLE GAP EVENT");
        break;
    default:
        break;
    }
    return 0;
}

timer_group_t timer_group = TIMER_GROUP_0;
timer_idx_t timer_idx = TIMER_0;

void timer_callback(void *arg)
{
    // Send notification to the task
    xTaskNotifyGive(xHandle);
}

void ble_app_advertise(void)
{
    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_DISC_LTD;
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;
    fields.name = (uint8_t *)ble_svc_gap_device_name();
    fields.name_len = strlen(ble_svc_gap_device_name());
    fields.name_is_complete = 1;
    ble_gap_adv_set_fields(&fields);
    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof adv_params);
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    timer_config_t config;
    config.alarm_en = TIMER_ALARM_EN;
    config.auto_reload = TIMER_AUTORELOAD_DIS;
    config.counter_dir = TIMER_COUNT_UP;
    config.divider = TIMG_T1_DIVIDER;
    config.intr_type = TIMER_INTR_LEVEL;
    config.counter_en = TIMER_PAUSE;
    timer_init(timer_group, timer_idx, &config);
    timer_set_alarm_value(timer_group, timer_idx, 15000000); // 15s in microsecond
    timer_enable_intr(timer_group, timer_idx);
    timer_isr_register(timer_group, timer_idx, timer_callback, NULL, ESP_INTR_FLAG_IRAM, NULL);

    // Start the timer
    while (1)
    {
        vTaskDelay(10);
        gpio_set_level(LED_PIN, 0);
        if (gpio_get_level(BUTTON_GPIO) == 0)
        {
            timer_start(timer_group, timer_idx);
            ble_gap_adv_start(ble_addr_type, NULL, 15000, &adv_params, ble_gap_event, NULL);
            xTaskCreate(&led_blink, "LED_BLINK", 512, NULL, 5, &xHandle);
            break;
        }
    }
}

void ble_app_on_sync(void)
{
    ble_addr_t addr;
    ble_hs_id_gen_rnd(1, &addr);
    ble_hs_id_set_rnd(addr.val);
    ble_hs_id_infer_auto(0, &ble_addr_type); // determines automatic address.
    ble_app_advertise();                     // start advertising the services -->
}

void host_task(void *param)
{
    nimble_port_run();
}

void try_to_connect_to_ble_and_exchange_data()
{
    nvs_flash_init();
    esp_nimble_hci_and_controller_init();                   // initialize bluetooth controller.
    nimble_port_init();                                     // nimble library initialization.
    ESP_ERROR_CHECK(ble_svc_gap_device_name_set(BLE_NAME)); // set BLE name.
    ble_svc_gap_init();                                     // initialize the gap service.
    ble_svc_gatt_init();                                    // initailize the gatt service.
    ble_gatts_count_cfg(gatt_svcs);                         // config all the gatt services that wanted to be used.
    ble_gatts_add_svcs(gatt_svcs);                          // queues all services.
    ble_hs_cfg.sync_cb = ble_app_on_sync;
    nimble_port_freertos_init(host_task);
}

#define BLINK_GPIO 2

// #define EXAMPLE_ESP_WIFI_SSID "Lukasz iPhone"
// #define EXAMPLE_ESP_WIFI_PASS "lukasz123"
#define EXAMPLE_ESP_MAXIMUM_RETRY 5
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN

static EventGroupHandle_t s_wifi_event_group;

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

static const char *TAG = "MQTT_EXAMPLE";
esp_mqtt_client_handle_t client;

void setup()
{
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(YL_69_ADC_CHANNEL, ADC_ATTEN_DB_11);
}

void save_no_wifi()
{
    float temperature;
    float humidity;
    uint16_t light;
    float moisture;

    turn_on_bh1750();
    esp_err_t ret = bh1750_init(I2C_NUM_0);
    if (ret != ESP_OK)
    {
        printf("BH1750 init failed: %d\n", ret);
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    ret = bh1750_measure(I2C_NUM_0, &light, BH1750_CONTINUOUS_HIGH_RES_MODE);
    if (ret == ESP_OK)
    {
        printf("Light intensity: %d lx\n", light);
    }
    else
    {
        printf("error while init bh1750");
    }

    uint32_t adc_reading = adc1_get_raw(YL_69_ADC_CHANNEL);
    // Przelicznie wartości na wilgotność gleby
    moisture = (adc_reading / 4095.0) * 100;
    printf("Soil moisture: %.2f%%\n", moisture);

    if (dht_read_float_data(SENSOR_TYPE, CONFIG_EXAMPLE_DATA_GPIO, &humidity, &temperature) == ESP_OK)
    {
        printf("Humidity: %.1f%% Temp: %.1fC\n", humidity, temperature);
    }
    else
        printf("Could not read data from sensor\n");

    printf("moisture before fixed: %f", moisture);
    int32_t temperature_fixed = (int32_t)(temperature * 100);
    int32_t humidity_fixed = (int32_t)(humidity * 100);
    int32_t light_fixed = (int32_t)(light * 100);
    int32_t moisture_fixed = (int32_t)(moisture * 100);
    printf("fixed moisture %d", moisture_fixed);

    // Zapis nowych danych do pamięci NVS
    nvs_handle handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &handle);
    if (err != ESP_OK)
    {
        printf("Error opening NVS handle!\n");
        return;
    }

    size_t data_count = 0;
    err = nvs_get_u32(handle, "data_count", &data_count);
    if (err != ESP_OK)
    {
        data_count = 0;
    }

    char key_temperature[30];
    sprintf(key_temperature, "dt_%d", data_count);
    char key_humidity[32];
    sprintf(key_humidity, "dh_%d", data_count);
    char key_light[32];
    sprintf(key_light, "dl_%d", data_count);
    char key_moisture[32];
    sprintf(key_moisture, "dm_%d", data_count);

    err = nvs_set_u32(handle, key_temperature, temperature_fixed);
    if (err != ESP_OK)
    {
        printf("Error saving temperature to NVS!\n");
    }
    err = nvs_set_u32(handle, key_humidity, humidity_fixed);
    if (err != ESP_OK)
    {
        printf("Error saving humidity to NVS!\n");
    }
    err = nvs_set_u32(handle, key_light, light_fixed);
    if (err != ESP_OK)
    {
        printf("Error saving light to NVS!\n");
    }
    err = nvs_set_u32(handle, key_moisture, moisture_fixed);
    if (err != ESP_OK)
    {
        printf("Error saving moisture to NVS!\n");
    }
    err = nvs_set_u32(handle, "data_count", ++data_count);
    if (err != ESP_OK)
    {
        printf("Error updating data count in NVS!\n");
    }
    printf("%d\n", data_count);

    uint32_t measurement_interval;
    err = nvs_get_u32(handle, "m-interval", &measurement_interval);
    if (err != ESP_OK)
    {
        printf("couldn't load interval");
    }

    nvs_commit(handle);
    nvs_close(handle);
    // esp_sleep_enable_ext1_wakeup((1u << WAKE_UP_BUTTON_GPIO) | (1u << RESET_BUTTON_GPIO), 0);
    esp_sleep_enable_ext0_wakeup(RESET_BUTTON_GPIO, 0);

    // Uśpienie płytki na 5 sekund
    esp_sleep_enable_timer_wakeup(time_to_measure * 1e6);
    esp_deep_sleep_start();
}

bool was_wifi_connected = false;
static int s_retry_num = 0;
static bool downloaded = false;

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    was_wifi_connected = false;
    printf("\n WIFI SSID: %s \n", wifi_credentials.ssid);
    printf(" WIFI password: %s \n", wifi_credentials.pass);

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY)
        {

            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        }
        else
        {
            wifi_event_sta_disconnected_t *event = (wifi_event_sta_disconnected_t *)event_data;
            char ssid_char[event->ssid_len];
            memcpy(event->ssid, ssid_char, sizeof(ssid_char));
            printf("%s", user_wifi_ssid);
            if (event->reason == WIFI_REASON_NO_AP_FOUND && strcmp(user_wifi_ssid, "ssid") == 0)
            {
                ESP_LOGI(TAG, "wrong ssid");
                xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            } else if (event->reason == WIFI_REASON_CONNECTION_FAIL)
            {
                ESP_LOGI(TAG, "wrong password");
                xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            }
            else if (event->reason == WIFI_REASON_NO_AP_FOUND)
            {
                ESP_LOGI(TAG, "ap no found");
                led_blink_no_wifi();
                save_no_wifi();
                // esp_sleep_enable_timer_wakeup(time_in_us*1e6);
                // printf("goigo to sleep for %d seconds\n", time_in_us);
                // vTaskDelay(100);
                // esp_deep_sleep_start();
            }
        }
        ESP_LOGI(TAG, "connect to the AP fail");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        was_wifi_connected = true;
        // reset_led();
    }
}

void erase_wifi_credentials_from_nvs()
{
    esp_err_t err;
    nvs_handle my_handle;
    err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE("NVS", "Error (%s) opening NVS handle!\n", esp_err_to_name(err));
        return;
    }
    err = nvs_erase_key(my_handle, "ssid");
    if (err != ESP_OK)
    {
        ESP_LOGE("NVS", "Error (%s) erasing ssid from NVS!\n", esp_err_to_name(err));
    }
    err = nvs_erase_key(my_handle, "pass");
    if (err != ESP_OK)
    {
        ESP_LOGE("NVS", "Error (%s) erasing pass from NVS!\n", esp_err_to_name(err));
    }
    nvs_close(my_handle);
    ESP_LOGI("NVS", "Wifi credentials erased successfully from NVS");
}

bool checkIsFirstConnection()
{
    bool isFirstConnection = false;
    nvs_handle my_handle;
    esp_err_t err = nvs_open("storage", NVS_READONLY, &my_handle);
    if (err != ESP_OK)
    {
        // ESP_LOG_INFO("Not init connection");
    }

    // Pobranie wartości flagi z pamięci NVS
    int flag;
    err = nvs_get_i32(my_handle, "flag", &flag);
    if (err != ESP_OK)
    {
        isFirstConnection = true;
    }

    // Zamknięcie pamięci NVS
    nvs_close(my_handle);
    return isFirstConnection;
}

void save_flag(int flag)
{
    nvs_handle my_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK)
    {
        // Obsługa błędu
    }

    err = nvs_set_i32(my_handle, "flag", flag);
    if (err != ESP_OK)
    {
        // Obsługa błędu
    }

    nvs_commit(my_handle);
    nvs_close(my_handle);
}
void erase_flag()
{
    nvs_handle my_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK)
    {
        // Obsługa błędu
    }

    err = nvs_erase_key(my_handle, "flag");
    if (err != ESP_OK)
    {
        // Obsługa błędu
    }

    nvs_commit(my_handle);
    nvs_close(my_handle);
}

void deactive_device() {
    esp_http_client_config_t config_delete = {
        .url = "http://172.20.10.2:8080/waterit/api/device/esp/BLE-PLANT/confirm",
        .method = HTTP_METHOD_DELETE,
    };
    esp_http_client_handle_t client_delete = esp_http_client_init(&config_delete);

    esp_err_t err = esp_http_client_perform(client_delete);
    if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "Status = %d, content_length = %d",
                 esp_http_client_get_status_code(client_delete),
                 esp_http_client_get_content_length(client_delete));
    }
    else
    {
        ESP_LOGE(TAG, "Error perform http request %s", esp_err_to_name(err));
        // TODO
    }
    esp_http_client_cleanup(client_delete);
}

void confirm_device() {
    esp_http_client_config_t config_delete = {
        .url = "http://172.20.10.2:8080/waterit/api/device/esp/BLE-PLANT/confirm",
        .method = HTTP_METHOD_POST,
    };
    esp_http_client_handle_t client_delete = esp_http_client_init(&config_delete);

    esp_err_t err = esp_http_client_perform(client_delete);
    if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "Status = %d, content_length = %d",
                 esp_http_client_get_status_code(client_delete),
                 esp_http_client_get_content_length(client_delete));
    }
    else
    {
        ESP_LOGE(TAG, "Error perform http request %s", esp_err_to_name(err));
        // TODO
    }
    esp_http_client_cleanup(client_delete);
}

void activate_device()
{
    esp_http_client_config_t config = {
        .url = "http://172.20.10.2:8080/waterit/api/device/activate",
        .method = HTTP_METHOD_POST,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);

    esp_http_client_set_post_field(client, "name=BLE-PLANT", strlen("name=BLE-PLANT"));
    vTaskDelay(10000 / portTICK_PERIOD_MS);
    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "Status = %d, content_length = %d",
                 esp_http_client_get_status_code(client),
                 esp_http_client_get_content_length(client));
    }
    else
    {
        ESP_LOGE(TAG, "Error perform http request %s", esp_err_to_name(err));
        erase_flag();
        erase_wifi_credentials_from_nvs();
        esp_restart();
    }
    esp_http_client_cleanup(client);
}

char *create_json_string(int data[][4], int num_objects)
{
    char *json_string;
    int string_length = 0;
    int i, j;

    // Calculate the length of the final JSON string
    string_length += 2; // for the opening and closing square brackets
    for (i = 0; i < num_objects; i++)
    {
        string_length += 1; // for the opening curly brace
        for (j = 0; j < 4; j++)
        {
            string_length += 22; // for the field name and colon
            string_length += 11; // for the maximum number of digits in an int
            if (j < 4 - 1)
            {
                string_length += 2; // for the comma and space
            }
        }
        string_length += 2; // for the closing curly brace and comma
    }
    string_length--; // remove the last comma

    // Allocate memory for the JSON string
    json_string = malloc(string_length + 10);
    char *header;
    header = malloc(100);
    nvs_handle handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &handle);
    uint32_t measurement_interval;
    err = nvs_get_u32(handle, "m-interval", &measurement_interval);
    if (err != ESP_OK)
    {
        printf("couldn't load interval");
    }

    nvs_close(handle);

    sprintf(header, "{ \"name\" : \"BLE-PLANT\", \"interval\" : %d,\n\"data\" : ", measurement_interval); // +1 for the null terminator
    strcpy(json_string, header);

    // Build the JSON string
    strcat(json_string, "[");
    for (i = 0; i < num_objects; i++)
    {
        strcat(json_string, "{");
        char field_name[50];
        sprintf(field_name, "\"lightIntensity\":%d,", data[i][0]);
        strcat(json_string, field_name);
        sprintf(field_name, "\"temperature\":%d,", data[i][1]);
        strcat(json_string, field_name);
        sprintf(field_name, "\"humidity\":%d,", data[i][2]);
        strcat(json_string, field_name);
        sprintf(field_name, "\"moistureHumidity\":%d", data[i][3]);
        strcat(json_string, field_name);
        strcat(json_string, "}");
        if (i < num_objects - 1)
        {
            strcat(json_string, ", ");
        }
    }
    strcat(json_string, "]}");

    return json_string;
}

void send_data_to_server(int *data, int size)
{

    char *json_data = create_json_string(data, size);

    printf("%s", json_data);

    esp_http_client_config_t config = {
        .url = "http://172.20.10.2:8080/waterit/api/device/history",
        .method = HTTP_METHOD_POST,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);

    esp_http_client_set_post_field(client, json_data, strlen(json_data));
    esp_http_client_set_header(client, "Content-Type", "application/json");

    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "Status = %d, content_length = %d",
                 esp_http_client_get_status_code(client),
                 esp_http_client_get_content_length(client));
    }
    else
    {
        ESP_LOGE(TAG, "Error perform http request %s", esp_err_to_name(err));
        // TODO
    }
    esp_http_client_cleanup(client);
}

void wifi_init_sta()
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

    printf("ssid before init: %s \n", wifi_credentials.ssid);
    printf("pssw before init: %s ", wifi_credentials.pass);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = {wifi_credentials.ssid},
            .password = {wifi_credentials.pass},
            /* Setting a password implies station will connect to all security modes including WEP/WPA.
             * However these modes are deprecated and not advisable to be used. Incase your Access point
             * doesn't support WPA2, these mode can be enabled by commenting below line */
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    strncpy((char *)wifi_config.sta.ssid, (char *)wifi_credentials.ssid, 32);     // copying new ssid to wifi_config
    strncpy((char *)wifi_config.sta.password, (char *)wifi_credentials.pass, 64); // copying new pass to wifi_config

    printf("\nWIFI SSID connect trial: %s ", (char *)wifi_config.sta.ssid);
    printf("\nWIFI PASSWORD connect trial: %s \n", (char *)wifi_config.sta.password);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT)
    {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 user_wifi_ssid, user_wifi_pass);

        if (checkIsFirstConnection())
        {
            save_flag(1);
            // activate_device();
        }
    }
    else if (bits & WIFI_FAIL_BIT)
    {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 user_wifi_ssid, user_wifi_pass);
        erase_wifi_credentials_from_nvs();
    }
    else
    {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }

    /* The event will not be processed after unregister */
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    // ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);
}

static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0)
    {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event)
{
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    // your_context_t *context = event->context;
    switch (event->event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        msg_id = esp_mqtt_client_publish(client, "/topic/qos1", "data_3", 0, 1, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "/topic/qos0", 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "/topic/qos1", 1);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "bh1750/resolution", 1);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "esp32/interval", 1);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "esp32/measurement-interval", 1);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "BLE-PLANT/reset", 1);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_unsubscribe(client, "/topic/qos1");
        ESP_LOGI(TAG, "sent unsubscribe successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        msg_id = esp_mqtt_client_publish(client, "/topic/qos0", "data", 0, 0, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        // printf("DATA=%.*s\r\n", event->data_len, event->data);
        // parse the received data and take appropriate action
        if (strcmp(event->topic, (char *)"bh1750/resolution") == 0)
        {
            printf("działa\n");
            printf("=%.*s\r \n", event->data_len, event->data);
            printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
            printf("DATA=%.*s\r\n", event->data_len, event->data);
            if (strncmp(event->data, "low", 3) == 0)
            {
                printf("jest low\n");
                is_low = true;
            }
            else if (strncmp(event->data, (char *)"high", 3) == 0)
            {
                printf("jest high\n");
                is_low = false;
            }
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
        if (strcmp(event->topic, (char *)"esp32/interval") == 0)
        {
            printf("działa\n");
            printf("=%.*s\r \n", event->data_len, event->data);
            printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
            printf("DATA=%.*s\r\n", event->data_len, event->data);
            int interval = atoi(event->data);
            time_in_us = interval * 1000000;
            printf("time in use: %d\n", interval);
            esp_sleep_enable_timer_wakeup(time_in_us);
            vTaskDelay(pdMS_TO_TICKS(2000));
            nvs_handle my_handle;
            esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
            if (err != ESP_OK)
            {
                // handle error
            }
            err = nvs_set_u32(my_handle, "p-interval", interval);
            if (err != ESP_OK)
            {
                // handle error
            }
            err = nvs_commit(my_handle);
            nvs_close(my_handle);
        }
        if (strcmp(event->topic, (char *)"esp32/measurement-interval") == 0)
        {
            printf("działa\n");
            printf("=%.*s\r \n", event->data_len, event->data);
            printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
            printf("DATA=%.*s\r\n", event->data_len, event->data);
            int interval = atoi(event->data);
            time_in_us = interval * 1000000;
            printf("time in use: %d\n", interval);
            esp_sleep_enable_timer_wakeup(time_in_us);
            nvs_handle my_handle;
            esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
            if (err != ESP_OK)
            {
                // handle error
            }
            err = nvs_set_u32(my_handle, "m-interval", interval);
            if (err != ESP_OK)
            {
                printf("couldn't set interval");
            }
            err = nvs_commit(my_handle);
            nvs_close(my_handle);
        }
        if (strncmp(event->topic, (char *)"BLE-PLANT/reset", 16) == 0)
        {
            printf("działa\n");
            printf("=%.*s\r \n", event->data_len, event->data);
            printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
            printf("DATA=%.*s\r\n", event->data_len, event->data);
            if (strncmp(event->data, "true", 4) == 0)
            {
                //  int msg_id;
                esp_mqtt_client_publish(client, "BLE-PLANT/reset", "false", 0, 1, 1);
                deactive_device();
                erase_wifi_credentials_from_nvs();
                nvs_flash_erase();
                erase_flag();
                vTaskDelay(pdMS_TO_TICKS(2000));
                esp_restart();
            }
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
        if (reset)
        {
            erase_wifi_credentials_from_nvs();
            reset = false;
        }
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT)
        {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno", event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
    return ESP_OK;
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    mqtt_event_handler_cb(event_data);
}

static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = "mqtt://172.20.10.2",
        .port = 1883,
    };
    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
    esp_mqtt_client_start(client);
}

void test(void *pvParameters)
{
    turn_on_bh1750();
    esp_err_t ret = bh1750_init(I2C_NUM_0);
    if (ret != ESP_OK)
    {
        printf("BH1750 init failed: %d\n", ret);
    }
    vTaskDelay(pdMS_TO_TICKS(1000));

#ifdef CONFIG_EXAMPLE_INTERNAL_PULLUP
    gpio_set_pull_mode(dht_gpio, GPIO_PULLUP_ONLY);
#endif

    while (1)
    {
        uint16_t lux;
        float temperature, humidity;
        char yl69_buffer[1024];

        if (is_low == false)
        {
            printf("measure high\n");
            ret = bh1750_measure(I2C_NUM_0, &lux, BH1750_CONTINUOUS_HIGH_RES_MODE);
        }
        else
        {
            printf("measure low\n");
            ret = bh1750_measure(I2C_NUM_0, &lux, BH1750_CONTINUOUS_LOW_RES_MODE);
        }

        if (ret == ESP_OK)
        {
            printf("Light intensity: %d lx\n", lux);
            char lux_char[50];
            sprintf(lux_char, "%d", lux);
            esp_mqtt_client_publish(client, "BLE-PLANT/lux", lux_char, 0, 1, 1);
        }
        else
        {
            printf("BH1750 measurement failed: %d\n", ret);
        }

        if (dht_read_float_data(SENSOR_TYPE, CONFIG_EXAMPLE_DATA_GPIO, &humidity, &temperature) == ESP_OK)
        {
            printf("Humidity: %.1f%% Temp: %.1fC\n", humidity, temperature);
            char temp_char[50];
            sprintf(temp_char, "%.1f", temperature);
            esp_mqtt_client_publish(client, "BLE-PLANT/temperature", temp_char, 0, 1, 1);
            char humidity_char[50];
            sprintf(humidity_char, "%.1f", humidity);
            esp_mqtt_client_publish(client, "BLE-PLANT/humidity", humidity_char, 0, 1, 1);
        }
        else
            printf("Could not read data from sensor\n");
        uint32_t adc_reading = adc1_get_raw(YL_69_ADC_CHANNEL);
        // Przelicznie wartości na wilgotność gleby
        float soil_moisture = (adc_reading / 4095.0) * 100;
        printf("Wilgotność gleby: %.2f%%\n", soil_moisture);
        char moisture_char[50];
        sprintf(moisture_char, "%.1f", soil_moisture);
        esp_mqtt_client_publish(client, "BLE-PLANT/moisture", moisture_char, 0, 1, 1);
        vTaskDelay(pdMS_TO_TICKS(2000));
        turn_off_bh1750();
        esp_sleep_enable_ext0_wakeup(RESET_BUTTON_GPIO, 0);

        // Uśpienie płytki na 5 sekund
        esp_sleep_enable_timer_wakeup(5000000);
        vTaskDelay(pdMS_TO_TICKS(150));
        esp_deep_sleep_start();
    }
}

static void yl69_task(void *arg)
{
    char yl69_buffer[1024];

    while (1)
    {
        uint32_t adc_reading = yl69_read();
        uint32_t adc_percentage = yl69_normalization(adc_reading);
        snprintf(yl69_buffer, sizeof(yl69_buffer), "{\"soil_mosture\": %d}", adc_percentage);
        printf("%s\n", yl69_buffer);

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}

void save_wifi_credentials_to_nvs(wifi_credentials_t wifi_credentials)
{
    esp_err_t err;
    nvs_handle my_handle;
    err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE("NVS", "Error (%s) opening NVS handle!\n", esp_err_to_name(err));
        return;
    }
    err = nvs_set_str(my_handle, "ssid", wifi_credentials.ssid);
    if (err != ESP_OK)
    {
        ESP_LOGE("NVS", "Error (%s) saving ssid to NVS!\n", esp_err_to_name(err));
    }
    err = nvs_set_str(my_handle, "pass", wifi_credentials.pass);
    if (err != ESP_OK)
    {
        ESP_LOGE("NVS", "Error (%s) saving pass to NVS!\n", esp_err_to_name(err));
    }
    nvs_commit(my_handle);
    nvs_close(my_handle);
    ESP_LOGI("NVS", "Wifi credentials saved successfully to NVS");
}

bool load_wifi_credentials_from_nvs(wifi_credentials_t *wifi_credentials)
{
    esp_err_t err;
    nvs_handle my_handle;
    err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE("NVS", "Error (%s) opening NVS handle!\n", esp_err_to_name(err));
        return false;
    }
    size_t required_size;
    err = nvs_get_str(my_handle, "ssid", NULL, &required_size);
    if (err != ESP_OK)
    {
        ESP_LOGE("NVS", "Error (%s) getting ssid from NVS!\n", esp_err_to_name(err));
        nvs_close(my_handle);
        return false;
    }
    err = nvs_get_str(my_handle, "ssid", wifi_credentials->ssid, &required_size);
    if (err != ESP_OK)
    {
        ESP_LOGE("NVS", "Error (%s) getting ssid from NVS!\n", esp_err_to_name(err));
        return false;
    }
    err = nvs_get_str(my_handle, "pass", NULL, &required_size);
    if (err != ESP_OK)
    {
        ESP_LOGE("NVS", "Error (%s) getting pass from NVS!\n", esp_err_to_name(err));
        nvs_close(my_handle);
        return false;
    }
    err = nvs_get_str(my_handle, "pass", wifi_credentials->pass, &required_size);
    if (err != ESP_OK)
    {
        ESP_LOGE("NVS", "Error (%s) getting pass from NVS!\n", esp_err_to_name(err));
        return false;
    }
    // err = nvs_erase_all(my_handle);
    // if (err != ESP_OK)
    // {
    //     ESP_LOGE("NVS", "Error (%s) earse all!\n", esp_err_to_name(err));
    //     return false;
    // }
    nvs_close(my_handle);
    ESP_LOGI("NVS", "Wifi credentials loaded successfully from NVS");
    return true;
}

void try_to_connect_to_wifi()
{

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        // ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    while (!were_ssid_given && !were_psk_given)
    {
        vTaskDelay(10);
    }
    were_ssid_given = false;
    were_psk_given = false;

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");

    wifi_init_sta();
    printf("Wifi connected status: " + was_wifi_connected);

    vTaskDelay(2000 / portTICK_RATE_MS);

    if (was_wifi_connected)
    {
        mqtt_app_start();
    }
    else
    {
        esp_restart();
        ESP_LOGI("ESP", "Restart esp bacause of lack of wifi connection..");
    }
}

void init_bluetooth_connect_if_first_time()
{
    if (!load_wifi_credentials_from_nvs(&wifi_credentials))
    {
        printf("\nConnect device by ble with esp sending ssid and password to WIFI network that you want to connect\n");
        try_to_connect_to_ble_and_exchange_data();
        while (!were_ssid_given && !were_psk_given)
        {
            vTaskDelay(100);
        }
        vTaskDelete(xHandle);
        xTaskCreate(&led_blink_connected, "LED_BLINK", 512, NULL, 5, &xHandle);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        vTaskDelete(xHandle);
        printf("before save ssid: %s\n", user_wifi_ssid);
        printf("before save pass: %s\n", user_wifi_pass);
        strcpy(wifi_credentials.ssid, user_wifi_ssid);
        strcpy(wifi_credentials.pass, user_wifi_pass);
        save_wifi_credentials_to_nvs(wifi_credentials);
        nvs_handle my_handle;
        esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
        if (err != ESP_OK)
        {
            printf("couldn't init storage");
        }
        err = nvs_set_u32(my_handle, "p-interval", time_in_us);
        if (err != ESP_OK)
        {
            printf("couldn't init p-interval");
        }
        err = nvs_set_u32(my_handle, "m-interval", time_to_measure);
        if (err != ESP_OK)
        {
            printf("couldn't init m-interval");
        }
        err = nvs_commit(my_handle);
        nvs_close(my_handle);
        try_to_connect_to_wifi();
        confirm_device();
        vTaskDelay(2000);
        esp_restart();
    }
    else
    {
        were_ssid_given = true;
        were_psk_given = true;
    }
}

void setup_sleep()
{
    // konfiguracja pinu wakeup
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_0, 0); // 0 - stan niski, 1 - stan wysoki
    printf("time in use: %d", time_in_us);
    esp_sleep_enable_timer_wakeup(time_in_us);
}

void set_up_button()
{
    gpio_config_t button_config = {
        .pin_bit_mask = 1LL << BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    gpio_config(&button_config);

    gpio_config_t wake_up_button_config = {
        .pin_bit_mask = 1LL << WAKE_UP_BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    gpio_config(&wake_up_button_config);

    gpio_config_t reset_button_config = {
        .pin_bit_mask = 1LL << RESET_BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    gpio_config(&reset_button_config);
}

void onWakeup()
{

    nvs_handle handle;
    esp_err_t err;
    size_t wakeup_count = 0;
    err = nvs_open("storage", NVS_READWRITE, &handle);
    err = nvs_get_u32(handle, "data_count", &wakeup_count);
    printf("wakeup_count: %d\n", wakeup_count);

    uint32_t push_interval;
    err = nvs_get_u32(handle, "p-interval", &push_interval);
    if (err != ESP_OK)
    {
        // handle error
    }

    uint32_t measurement_interval;
    err = nvs_get_u32(handle, "m-interval", &measurement_interval);
    if (err != ESP_OK)
    {
        printf("couldn't load interval");
    }

    nvs_close(handle);
    printf("%d", measurement_interval);

    // Jeśli minęło już 5 minut
    if (wakeup_count >= push_interval / measurement_interval)
    {

        try_to_connect_to_wifi();
        // Odczyt danych z pamięci NVS
        err = nvs_open("storage", NVS_READWRITE, &handle);
        if (err != ESP_OK)
        {
            printf("Error opening NVS handle!\n");
            return;
        }

        int data[wakeup_count][4];

        // Odczytanie danych z wektora z pamięci NVS i wyświetlenie ich
        for (size_t i = 0; i < wakeup_count; i++)
        {
            uint32_t temperature, humidity, light, moisture;
            char key_temperature[30];
            sprintf(key_temperature, "dt_%d", i);
            err = nvs_get_u32(handle, key_temperature, &temperature);
            if (err != ESP_OK)
            {
                printf("Error reading temperature from NVS!\n");
            }
            char key_humidity[30];
            sprintf(key_humidity, "dh_%d", i);
            err = nvs_get_u32(handle, key_humidity, &humidity);
            if (err != ESP_OK)
            {
                printf("Error reading humidity from NVS!\n");
            }
            printf("Temperature: %f°C, Humidity: %f%%\n", (float)temperature / 100, (float)humidity / 100);

            char key_light[30];
            sprintf(key_light, "dl_%d", i);
            err = nvs_get_u32(handle, key_light, &light);
            if (err != ESP_OK)
            {
                printf("Error reading light from NVS!\n");
            }
            printf("Light intensity %d lx\n", light);

            char key_moisture[30];
            sprintf(key_moisture, "dm_%d", i);
            err = nvs_get_u32(handle, key_moisture, &moisture);
            if (err != ESP_OK)
            {
                printf("Error reading moisture from NVS!\n");
            }
            printf("Soil moisture %d%%\n", moisture);

            data[i][0] = light;
            data[i][1] = temperature;
            data[i][2] = humidity;
            data[i][3] = moisture;
        }

        send_data_to_server((int *)data, wakeup_count);

        // Czyszczenie wektora danych w pamięci NVS
        err = nvs_erase_key(handle, "data_count");
        if (err != ESP_OK)
        {
            printf("Error erasing data count from NVS!\n");
        }
        for (size_t i = 0; i < wakeup_count; i++)
        {
            char key_temperature[32];
            sprintf(key_temperature, "dt_%d", i);
            char key_humidity[32];
            sprintf(key_humidity, "dh_%d", i);
            char key_light[32];
            sprintf(key_light, "dl_%d", i);
            char key_moisture[32];
            sprintf(key_moisture, "dm_%d", i);
            err = nvs_erase_key(handle, key_temperature);
            if (err != ESP_OK)
            {
                printf("Error erasing temp from NVS!\n");
            }
            err = nvs_erase_key(handle, key_humidity);
            if (err != ESP_OK)
            {
                printf("Error erasing humidity from NVS!\n");
            }
            err = nvs_erase_key(handle, key_light);
            if (err != ESP_OK)
            {
                printf("Error erasing light from NVS!\n");
            }
            err = nvs_erase_key(handle, key_moisture);
            if (err != ESP_OK)
            {
                printf("Error erasing moisture from NVS!\n");
            }
        }
        nvs_commit(handle);
        nvs_close(handle);

        wakeup_count = 0;
    }
}

void save()
{
    // Pobranie danych z czujnika DHT22
    float temperature;
    float humidity;
    uint16_t light;
    float moisture;

    turn_on_bh1750();
    esp_err_t ret = bh1750_init(I2C_NUM_0);
    if (ret != ESP_OK)
    {
        printf("BH1750 init failed: %d\n", ret);
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    ret = bh1750_measure(I2C_NUM_0, &light, BH1750_CONTINUOUS_HIGH_RES_MODE);
    if (ret == ESP_OK)
    {
        printf("Light intensity: %d lx\n", light);
    }
    else
    {
        printf("error while init bh1750");
    }

    uint32_t adc_reading = adc1_get_raw(YL_69_ADC_CHANNEL);
    // Przelicznie wartości na wilgotność gleby
    moisture = (adc_reading / 4095.0) * 100;
    printf("Soil moisture: %.2f%%\n", moisture);

    if (dht_read_float_data(SENSOR_TYPE, CONFIG_EXAMPLE_DATA_GPIO, &humidity, &temperature) == ESP_OK)
    {
        printf("Humidity: %.1f%% Temp: %.1fC\n", humidity, temperature);
    }
    else
        printf("Could not read data from sensor\n");

    printf("moisture before fixed: %f", moisture);
    int32_t temperature_fixed = (int32_t)(temperature * 100);
    int32_t humidity_fixed = (int32_t)(humidity * 100);
    int32_t light_fixed = (int32_t)(light * 100);
    int32_t moisture_fixed = (int32_t)(moisture * 100);
    printf("fixed moisture %d", moisture_fixed);

    // Zapis nowych danych do pamięci NVS
    nvs_handle handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &handle);
    if (err != ESP_OK)
    {
        printf("Error opening NVS handle!\n");
        return;
    }

    size_t data_count = 0;
    err = nvs_get_u32(handle, "data_count", &data_count);
    if (err != ESP_OK)
    {
        data_count = 0;
    }

    char key_temperature[30];
    sprintf(key_temperature, "dt_%d", data_count);
    char key_humidity[32];
    sprintf(key_humidity, "dh_%d", data_count);
    char key_light[32];
    sprintf(key_light, "dl_%d", data_count);
    char key_moisture[32];
    sprintf(key_moisture, "dm_%d", data_count);

    err = nvs_set_u32(handle, key_temperature, temperature_fixed);
    if (err != ESP_OK)
    {
        printf("Error saving temperature to NVS!\n");
    }
    err = nvs_set_u32(handle, key_humidity, humidity_fixed);
    if (err != ESP_OK)
    {
        printf("Error saving humidity to NVS!\n");
    }
    err = nvs_set_u32(handle, key_light, light_fixed);
    if (err != ESP_OK)
    {
        printf("Error saving light to NVS!\n");
    }
    err = nvs_set_u32(handle, key_moisture, moisture_fixed);
    if (err != ESP_OK)
    {
        printf("Error saving moisture to NVS!\n");
    }
    err = nvs_set_u32(handle, "data_count", ++data_count);
    if (err != ESP_OK)
    {
        printf("Error updating data count in NVS!\n");
    }
    printf("%d\n", data_count);

    uint32_t measurement_interval;
    err = nvs_get_u32(handle, "m-interval", &measurement_interval);
    if (err != ESP_OK)
    {
        printf("couldn't load interval");
    }

    nvs_commit(handle);
    nvs_close(handle);
    // esp_sleep_enable_ext1_wakeup((1u << WAKE_UP_BUTTON_GPIO) | (1u << RESET_BUTTON_GPIO), 0);
    esp_sleep_enable_ext0_wakeup(RESET_BUTTON_GPIO, 0);

    // Uśpienie płytki na 5 sekund
    esp_sleep_enable_timer_wakeup(measurement_interval * 1e6);
    esp_deep_sleep_start();
}

void task_wake_up_button(void *pvParameter)
{
    for (;;)
    {
        if (gpio_get_level(WAKE_UP_BUTTON_GPIO) == 0)
        {
            wake_up = true;
        }

        vTaskDelay(10);
    }
}

void task_reset_button(void *pvParameter)
{
    for (;;)
    {
        if (gpio_get_level(RESET_BUTTON_GPIO) == 0)
        {
            reset = true;
        }

        vTaskDelay(10);
    }
}

void delete_device()
{
    deactive_device();
    esp_http_client_config_t config_delete = {
        .url = "http://172.20.10.2:8080/waterit/api/device/esp/BLE-PLANT",
        .method = HTTP_METHOD_DELETE,
    };
    esp_http_client_handle_t client_delete = esp_http_client_init(&config_delete);

    esp_err_t err = esp_http_client_perform(client_delete);
    if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "Status = %d, content_length = %d",
                 esp_http_client_get_status_code(client_delete),
                 esp_http_client_get_content_length(client_delete));
    }
    else
    {
        ESP_LOGE(TAG, "Error perform http request %s", esp_err_to_name(err));
        // TODO
    }
    esp_http_client_cleanup(client_delete);
}

void app_main(void)
{
    xTaskCreatePinnedToCore(task_wake_up_button, "wake-up", configMINIMAL_STACK_SIZE * 5, NULL, 5, NULL, APP_CPU_NUM);
    xTaskCreatePinnedToCore(task_reset_button, "reset", configMINIMAL_STACK_SIZE * 5, NULL, 5, NULL, APP_CPU_NUM);
    // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES)
    {
        // ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    led_blink_setup();
    set_up_button();
    setup();
    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode,
                       I2C_MASTER_RX_BUF_DISABLE,
                       I2C_MASTER_TX_BUF_DISABLE, 0);

    init_bluetooth_connect_if_first_time();
    if (reset)
    {
        led_blink_wake_up();
        try_to_connect_to_wifi();
        led_blink_delete();
        printf("jak?");
        delete_device();
        nvs_flash_erase();
        esp_restart();
    }
    printf("%s", wake_up ? "true" : "false");
    if (wake_up)
    {
        led_blink_wake_up();
        printf("wake-up via button");
        try_to_connect_to_wifi();
        xTaskCreatePinnedToCore(test, "test", configMINIMAL_STACK_SIZE * 5, NULL, 5, NULL, APP_CPU_NUM);
    }
    else
    {
        onWakeup();
        save();
    }
}
