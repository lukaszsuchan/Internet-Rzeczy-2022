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
#include <bh1750.h>
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

volatile char *user_wifi_ssid = "ssid";
volatile char *user_wifi_pass = "password";
volatile char *main_topic = "waterit/device1/parameter/";

// ble config
#define DEVICE_INFO_SERVICE_UUID 0x180A

#define MANUFACTURER_NAME_CHAR 0xFEF4

#define SSID_CHAR 0xDEAD
#define PSK_CHAR 0xDEAE

#define BLE_NAME "BLE-PLANT"

char *TAG2 = "BLE-CONNECTION";

uint8_t ble_addr_type;

volatile bool were_ssid_given = false;
volatile bool were_psk_given = false;

// ble implementation

void ble_app_advertise(void);

// callback from characteristic 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff
static int device_write_ssid(uint8_t conn_handle, uint8_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
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
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    ble_gap_adv_start(ble_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL);
}

void ble_app_on_sync(void)
{
    // ble_addr_t addr;
    // ble_hs_id_gen_rnd(1, &addr);
    // ble_hs_id_set_rnd(addr.val);
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
#define EXAMPLE_ESP_MAXIMUM_RETRY 10
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN

static EventGroupHandle_t s_wifi_event_group;

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

static const char *TAG = "MQTT_EXAMPLE";
esp_mqtt_client_handle_t client;

void setup()
{
    yl69_setup(ADC_CHANNEL_1);
}

bool was_wifi_connected = false;
static int s_retry_num = 0;
static bool downloaded = false;

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    was_wifi_connected = false;
    printf("\n WIFI SSID: %s \n", user_wifi_ssid);
    printf(" WIFI password: %s \n", user_wifi_pass);

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY)
        {

            // xTaskCreate(&led_blink, "LED_BLINK", 512, NULL, 5, NULL);
            // vTaskDelay(1000/portTICK_RATE_MS);
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        }
        else
        {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
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

    printf("ssid before init: %s \n", user_wifi_ssid);
    printf("pssw before init: %s ", user_wifi_pass);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = {user_wifi_ssid},
            .password = {user_wifi_pass},
            /* Setting a password implies station will connect to all security modes including WEP/WPA.
             * However these modes are deprecated and not advisable to be used. Incase your Access point
             * doesn't support WPA2, these mode can be enabled by commenting below line */
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    strncpy((char *)wifi_config.sta.ssid, (char *)user_wifi_ssid, 32);     // copying new ssid to wifi_config
    strncpy((char *)wifi_config.sta.password, (char *)user_wifi_pass, 32); // copying new pass to wifi_config

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
    }
    else if (bits & WIFI_FAIL_BIT)
    {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 user_wifi_ssid, user_wifi_pass);
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
        printf("DATA=%.*s\r\n", event->data_len, event->data);
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
        .uri = "mqtt://192.168.0.81",
        .port = 1883,
    };
    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
    esp_mqtt_client_start(client);
}

void test(void *pvParameters)
{
    i2c_dev_t dev;
    memset(&dev, 0, sizeof(i2c_dev_t)); // Zero descriptor

    bh1750_init_desc(&dev, ADDR, 0, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL);
    bh1750_setup(&dev, BH1750_MODE_CONTINUOUS, BH1750_RES_HIGH);

#ifdef CONFIG_EXAMPLE_INTERNAL_PULLUP
    gpio_set_pull_mode(dht_gpio, GPIO_PULLUP_ONLY);
#endif

    while (1)
    {
        uint16_t lux;
        float temperature, humidity;

        if (bh1750_read(&dev, &lux) != ESP_OK)
            printf("Could not read lux data\n");
        else
        {
            printf("Lux: %d\n", lux);
            char lux_char[50];
            sprintf(lux_char, "%d", lux);
            esp_mqtt_client_publish(client, "lux", lux_char, 0, 1, 0);
        }
        if (dht_read_float_data(SENSOR_TYPE, CONFIG_EXAMPLE_DATA_GPIO, &humidity, &temperature) == ESP_OK)
        {
            printf("Humidity: %.1f%% Temp: %.1fC\n", humidity, temperature);
            char temp_char[50];
            sprintf(temp_char, "%.1f", temperature);
            esp_mqtt_client_publish(client, "temperature", temp_char, 0, 1, 0);
            char humidity_char[50];
            sprintf(humidity_char, "%.1f", humidity);
            esp_mqtt_client_publish(client, "humidity", humidity_char, 0, 1, 0);
        }
        else
            printf("Could not read data from sensor\n");
        vTaskDelay(pdMS_TO_TICKS(3000));
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

struct wifi_credentials

void save_wifi_config_to_nvs(wifi_config_t wifi_config)
{
    nvs_handle handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK)
    {
        ESP_LOGE("NVS", "Error (%s) opening NVS handle!\n", esp_err_to_name(err));
        return;
    }
    err = nvs_set_str(handle, NVS_WIFI_SSID, (const char *)wifi_config.sta.ssid);
    if (err != ESP_OK)
    {
        ESP_LOGE("NVS", "Error (%s) setting SSID in NVS!\n", esp_err_to_name(err));
    }
    err = nvs_set_str(handle, NVS_WIFI_PASS, (const char *)wifi_config.sta.password);
    if (err != ESP_OK)
    {
        ESP_LOGE("NVS", "Error (%s) setting password in NVS!\n", esp_err_to_name(err));
    }
    err = nvs_commit(handle);
    if (err != ESP_OK)
    {
        ESP_LOGE("NVS", "Error (%s) committing changes to NVS!\n", esp_err_to_name(err));
    }
    nvs_close(handle);
}

bool load_wifi_config_from_nvs(wifi_config_t *wifi_config)
{
    nvs_handle handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK)
    {
        ESP_LOGE("NVS", "Error (%s) opening NVS handle!\n", esp_err_to_name(err));
        return;
    }
    size_t ssid_size = sizeof(wifi_config->sta.ssid);
    err = nvs_get_str(handle, NVS_WIFI_SSID, (char *)wifi_config->sta.ssid, &ssid_size);
    if (err != ESP_OK)
    {
        ESP_LOGE("NVS", "Error (%s) getting SSID from NVS!\n", esp_err_to_name(err));
        return false;
    }
    size_t pass_size = sizeof(wifi_config->sta.password);
    err = nvs_get_str(handle, NVS_WIFI_PASS, (char *)wifi_config->sta.password, &pass_size);
    if (err != ESP_OK)
    {
        ESP_LOGE("NVS", "Error (%s) getting password from NVS!\n", esp_err_to_name(err));
    }
    nvs_close(handle);
    return true;
}

void try_to_connect_to_wifi()
{
    // xTaskCreate(&turn_on_blue_led, "BLUE_LED_SHINING", 512, NULL, 5, NULL);

    printf("\nConnect device by ble with esp sending ssid and password to WIFI network that you want to connect\n");
    try_to_connect_to_ble_and_exchange_data();

    while (!were_ssid_given && !were_psk_given)
    {
        vTaskDelay(10);
    }
    were_ssid_given = false;
    were_psk_given = false;

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

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

void app_main(void)
{
    setup();
    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
    esp_log_level_set("MQTT_EXAMPLE", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_TCP", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_SSL", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);

    // ESP_ERROR_CHECK(nvs_flash_init());
    // ESP_ERROR_CHECK(esp_netif_init());
    // ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    // wifi_init_sta();

    // mqtt_app_start();
    i2cdev_init(); // Init library
    try_to_connect_to_wifi();

    // xTaskCreate(yl69_task, "yl69_task", 4*1024, NULL, 1, NULL);
    xTaskCreatePinnedToCore(test, "test", configMINIMAL_STACK_SIZE * 5, NULL, 5, NULL, APP_CPU_NUM);
}
