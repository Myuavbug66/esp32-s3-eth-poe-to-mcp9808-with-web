/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
/*
* Temperture pouput String 
* 458 for monitor
* 267-289 For WEB
* 303 for JSON API
*/
/* ESP32-S3 PoE temperature monitor using a DS18B20 1-Wire sensor. */
#include <stdio.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_mac.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_eth.h"
#include "esp_eth_phy_w5500.h"
#include "esp_eth_mac_w5500.h"
#include "esp_http_server.h"
#include "esp_ota_ops.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "onewire_bus.h"
#include "ds18b20.h"
#include "mdns.h"
#include "lwip/apps/netbiosns.h"

static const char *TAG = "ds18b20";

#define DEVICE_HOSTNAME             "esp32-s3-eth"
#define DS18B20_GPIO                CONFIG_DS18B20_GPIO
#define DS18B20_TEMP_OFFSET_F_DEFAULT  -1.0f
#define DS18B20_TEMP_OFFSET_STEP_F     0.1f
#define SAMPLE_INTERVAL_MS          1000        /*!< Time between sensor reads */
#define SAMPLES_PER_AVERAGE         5           /*!< Number of reads averaged into each logged value (5 sec) */
#define MAX_DS18B20_SENSORS         2

/* Shared temperature/offset state, protected by g_data_mutex since it is written
 * from the sensor task and read/written from HTTP server request handlers. */
static SemaphoreHandle_t g_data_mutex;
static float g_temp_f = 0.0f;
static float g_temp2_f = 0.0f;
static float g_offset_f = DS18B20_TEMP_OFFSET_F_DEFAULT;
static SemaphoreHandle_t g_telnet_mutex;
static int g_telnet_client = -1;
static vprintf_like_t g_usb_vprintf;

#define TELNET_PORT 23

static int telnet_vprintf(const char *format, va_list args)
{
    va_list copy;
    va_copy(copy, args);
    int result = g_usb_vprintf(format, args);

    char line[512];
    int length = vsnprintf(line, sizeof(line), format, copy);
    va_end(copy);
    if (length <= 0) {
        return result;
    }
    if (length >= sizeof(line)) {
        length = sizeof(line) - 1;
    }

    xSemaphoreTake(g_telnet_mutex, portMAX_DELAY);
    if (g_telnet_client >= 0) {
        int sent = send(g_telnet_client, line, length, MSG_DONTWAIT);
        if (sent < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            close(g_telnet_client);
            g_telnet_client = -1;
        }
    }
    xSemaphoreGive(g_telnet_mutex);
    return result;
}

static void telnet_server_task(void *arg)
{
    int server_socket = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (server_socket < 0) {
        ESP_LOGE(TAG, "Could not create Telnet socket");
        vTaskDelete(NULL);
        return;
    }

    int reuse = 1;
    setsockopt(server_socket, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    struct sockaddr_in server_address = {
        .sin_family = AF_INET,
        .sin_port = htons(TELNET_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    if (bind(server_socket, (struct sockaddr *)&server_address, sizeof(server_address)) < 0 ||
        listen(server_socket, 1) < 0) {
        ESP_LOGE(TAG, "Could not start Telnet server");
        close(server_socket);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Telnet log server listening on port %d", TELNET_PORT);
    while (1) {
        struct sockaddr_in client_address;
        socklen_t address_length = sizeof(client_address);
        int client_socket = accept(server_socket, (struct sockaddr *)&client_address, &address_length);
        if (client_socket < 0) {
            continue;
        }

        fcntl(client_socket, F_SETFL, O_NONBLOCK);
        xSemaphoreTake(g_telnet_mutex, portMAX_DELAY);
        if (g_telnet_client >= 0) {
            close(g_telnet_client);
        }
        g_telnet_client = client_socket;
        const char welcome[] = "ESP32-S3 PoE log monitor\r\n";
        send(client_socket, welcome, sizeof(welcome) - 1, MSG_DONTWAIT);
        xSemaphoreGive(g_telnet_mutex);

        while (1) {
            char input;
            int received = recv(client_socket, &input, 1, MSG_DONTWAIT);
            if (received == 0 || (received < 0 && errno != EAGAIN && errno != EWOULDBLOCK)) {
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        xSemaphoreTake(g_telnet_mutex, portMAX_DELAY);
        if (g_telnet_client == client_socket) {
            close(g_telnet_client);
            g_telnet_client = -1;
        }
        xSemaphoreGive(g_telnet_mutex);
    }
}

static void start_telnet_server(void)
{
    g_telnet_mutex = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(g_telnet_mutex != NULL ? ESP_OK : ESP_ERR_NO_MEM);
    g_usb_vprintf = esp_log_set_vprintf(telnet_vprintf);
    xTaskCreate(telnet_server_task, "telnet_server", 4096, NULL, 5, NULL);
}

/**
 * @brief Log PoE Ethernet link/IP events
 */
static void eth_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == ETH_EVENT) {
        switch (event_id) {
        case ETHERNET_EVENT_CONNECTED:
            ESP_LOGI(TAG, "PoE Ethernet link up");
            break;
        case ETHERNET_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "PoE Ethernet link down");
            break;
        default:
            break;
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_ETH_GOT_IP) {
        const ip_event_got_ip_t *event = (const ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "PoE Ethernet got IP: " IPSTR, IP2STR(&event->ip_info.ip));
    }
}

/**
 * @brief Bring up the PoE-powered W5500 SPI Ethernet interface with a static IP
 */
static void ethernet_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &eth_event_handler, NULL));

    /* Required by the W5500 driver to attach its INT GPIO interrupt handler */
    esp_err_t isr_err = gpio_install_isr_service(0);
    if (isr_err != ESP_OK && isr_err != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(isr_err);
    }

    /* Hardware-reset the W5500 before talking to it over SPI */
    gpio_config_t rst_cfg = {
        .pin_bit_mask = 1ULL << CONFIG_ETH_SPI_RST_GPIO,
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&rst_cfg));
    gpio_set_level(CONFIG_ETH_SPI_RST_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(CONFIG_ETH_SPI_RST_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(50));

    spi_bus_config_t buscfg = {
        .miso_io_num = CONFIG_ETH_SPI_MISO_GPIO,
        .mosi_io_num = CONFIG_ETH_SPI_MOSI_GPIO,
        .sclk_io_num = CONFIG_ETH_SPI_SCLK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t spi_devcfg = {
        .mode = 0,
        .clock_speed_hz = CONFIG_ETH_SPI_CLOCK_MHZ * 1000 * 1000,
        .queue_size = 20,
        .spics_io_num = CONFIG_ETH_SPI_CS_GPIO,
    };

    eth_w5500_config_t w5500_config = ETH_W5500_DEFAULT_CONFIG(SPI2_HOST, &spi_devcfg);
    /* Poll instead of relying on the INT GPIO, to rule out INT wiring/config as a cause of RX issues */
    w5500_config.base.int_gpio_num = -1;
    w5500_config.base.poll_period_ms = 10;

    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    esp_eth_mac_t *mac = esp_eth_mac_new_w5500(&w5500_config, &mac_config);

    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    esp_eth_phy_t *phy = esp_eth_phy_new_w5500(&phy_config);

    esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy);
    esp_eth_handle_t eth_handle = NULL;
    ESP_ERROR_CHECK(esp_eth_driver_install(&eth_config, &eth_handle));

    /* Generic W5500 modules often ship with an unprogrammed (all-zero) MAC,
     * so assign one derived from the ESP32's own efuse-programmed base MAC. */
    uint8_t mac_addr[6];
    ESP_ERROR_CHECK(esp_read_mac(mac_addr, ESP_MAC_ETH));
    ESP_ERROR_CHECK(esp_eth_ioctl(eth_handle, ETH_CMD_S_MAC_ADDR, mac_addr));
    ESP_LOGI(TAG, "Ethernet MAC Address: %02X:%02X:%02X:%02X:%02X:%02X",
             mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);

    esp_netif_config_t netif_cfg = ESP_NETIF_DEFAULT_ETH();
    esp_netif_t *eth_netif = esp_netif_new(&netif_cfg);
    ESP_ERROR_CHECK(esp_netif_attach(eth_netif, esp_eth_new_netif_glue(eth_handle)));
    ESP_ERROR_CHECK(esp_netif_set_hostname(eth_netif, DEVICE_HOSTNAME));

    ESP_ERROR_CHECK(esp_netif_dhcpc_stop(eth_netif));
    esp_netif_ip_info_t ip_info = { 0 };
    ip_info.ip.addr = esp_ip4addr_aton(CONFIG_ETH_STATIC_IP_ADDR);
    ip_info.gw.addr = esp_ip4addr_aton(CONFIG_ETH_STATIC_GW_ADDR);
    ip_info.netmask.addr = esp_ip4addr_aton(CONFIG_ETH_STATIC_NETMASK_ADDR);
    ESP_ERROR_CHECK(esp_netif_set_ip_info(eth_netif, &ip_info));

    ESP_ERROR_CHECK(esp_eth_start(eth_handle));

    /* Advertise the hostname via mDNS so it shows up on network scans/browsers */
    ESP_ERROR_CHECK(mdns_init());
    ESP_ERROR_CHECK(mdns_hostname_set(DEVICE_HOSTNAME));
    ESP_ERROR_CHECK(mdns_instance_name_set("ESP32-S3 Room Temperature" ));
    ESP_ERROR_CHECK(mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0));
    /* NetBIOS responder: shows the hostname in Windows Network view and most LAN scanners */
    netbiosns_init();
    netbiosns_set_name(DEVICE_HOSTNAME);}

static esp_err_t favicon_get_handler(httpd_req_t *req)
{
    /* Avoid noisy 404 logs from browsers auto-requesting /favicon.ico */
    httpd_resp_set_status(req, "204 No Content");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

static esp_err_t root_get_handler(httpd_req_t *req)
{
    float temp_f, temp2_f, offset_f;
    xSemaphoreTake(g_data_mutex, portMAX_DELAY);
    temp_f = g_temp_f;
    temp2_f = g_temp2_f;
    offset_f = g_offset_f;
    xSemaphoreGive(g_data_mutex);

    char resp[4096];
    int len = snprintf(resp, sizeof(resp),
        "<!DOCTYPE html><html><head>"
        "<title>Room Temperatures 1</title>"
        "<style>body{font-family:sans-serif;text-align:center;margin-top:40px}"
        ".temp{font-size:64px}"
        ".btn{font-size:28px;padding:10px 26px;margin:10px;text-decoration:none;"
        "border:1px solid #333;border-radius:8px;display:inline-block;color:#000;"
        "background:none;cursor:pointer}</style>"
        "</head><body>"
        "<h1>Room Temperatures 1</h1>"
        "<h2>Sensor 1</h2><div class=\"temp\" id=\"temp\">%.2f&nbsp;&deg;F</div>"
        "<h2>Sensor 2</h2><div class=\"temp\" id=\"temp2\">%.2f&nbsp;&deg;F</div>"
        "<p>Calibration offset: <span id=\"offset\">%.2f</span>&nbsp;&deg;F</p>"
        "<button class=\"btn\" onclick=\"adjust('/dec')\">-</button>"
        "<button class=\"btn\" onclick=\"adjust('/inc')\">+</button>"
        "<script>"
        "function update(d){document.getElementById('temp').innerHTML=d.temp_f.toFixed(2)+'&nbsp;&deg;F';"
        "document.getElementById('temp2').innerHTML=d.temp2_f.toFixed(2)+'&nbsp;&deg;F';"
        "document.getElementById('offset').textContent=d.offset_f.toFixed(2);}"
        "function poll(){fetch('/api/temp').then(r=>r.json()).then(update);}"
        "function adjust(url){fetch(url).then(r=>r.json()).then(update);}"
        "setInterval(poll,2000);"
        "</script>"
        "</body></html>",
        temp_f, temp2_f, offset_f);
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, resp, len);
}

static esp_err_t api_temp_get_handler(httpd_req_t *req)
{
    float temp_f, temp2_f, offset_f;
    xSemaphoreTake(g_data_mutex, portMAX_DELAY);
    temp_f = g_temp_f;
    temp2_f = g_temp2_f;
    offset_f = g_offset_f;
    xSemaphoreGive(g_data_mutex);

    char resp[160];
    int len = snprintf(resp, sizeof(resp), "{\"temp_f\":%.2f,\"temp2_f\":%.2f,\"offset_f\":%.2f}",
                       temp_f, temp2_f, offset_f);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, resp, len);
}

static esp_err_t offset_inc_handler(httpd_req_t *req)
{
    xSemaphoreTake(g_data_mutex, portMAX_DELAY);
    g_offset_f += DS18B20_TEMP_OFFSET_STEP_F;
    xSemaphoreGive(g_data_mutex);
    return api_temp_get_handler(req);
}

static esp_err_t offset_dec_handler(httpd_req_t *req)
{
    xSemaphoreTake(g_data_mutex, portMAX_DELAY);
    g_offset_f -= DS18B20_TEMP_OFFSET_STEP_F;
    xSemaphoreGive(g_data_mutex);
    return api_temp_get_handler(req);
}

static esp_err_t ota_post_handler(httpd_req_t *req)
{
    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);
    if (update_partition == NULL) {
        ESP_LOGE(TAG, "No OTA partition available");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No OTA partition available");
        return ESP_FAIL;
    }

    esp_ota_handle_t ota_handle;
    esp_err_t err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OTA begin failed: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA begin failed");
        return err;
    }

    char buffer[4096];
    int remaining = req->content_len;
    while (remaining > 0) {
        int received = httpd_req_recv(req, buffer, remaining > sizeof(buffer) ? sizeof(buffer) : remaining);
        if (received <= 0) {
            err = received == HTTPD_SOCK_ERR_TIMEOUT ? ESP_ERR_TIMEOUT : ESP_FAIL;
            break;
        }
        err = esp_ota_write(ota_handle, buffer, received);
        if (err != ESP_OK) {
            break;
        }
        remaining -= received;
    }

    if (err == ESP_OK && remaining == 0) {
        err = esp_ota_end(ota_handle);
    } else {
        esp_ota_abort(ota_handle);
    }

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OTA upload failed: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA upload failed");
        return err;
    }

    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not select OTA partition: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Could not select OTA partition");
        return err;
    }

    ESP_LOGI(TAG, "OTA complete; rebooting into %s", update_partition->label);
    httpd_resp_sendstr(req, "OTA complete. Rebooting...");
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();
    return ESP_OK;
}

static void start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    /* Close the oldest idle connection instead of refusing new ones once max_open_sockets is hit */
    config.lru_purge_enable = true;
    httpd_handle_t server = NULL;
    ESP_ERROR_CHECK(httpd_start(&server, &config));

    const httpd_uri_t root_uri = { .uri = "/", .method = HTTP_GET, .handler = root_get_handler };
    const httpd_uri_t favicon_uri = { .uri = "/favicon.ico", .method = HTTP_GET, .handler = favicon_get_handler };
    const httpd_uri_t api_temp_uri = { .uri = "/api/temp", .method = HTTP_GET, .handler = api_temp_get_handler };
    const httpd_uri_t inc_uri = { .uri = "/inc", .method = HTTP_GET, .handler = offset_inc_handler };
    const httpd_uri_t dec_uri = { .uri = "/dec", .method = HTTP_GET, .handler = offset_dec_handler };
    const httpd_uri_t ota_uri = { .uri = "/ota", .method = HTTP_POST, .handler = ota_post_handler };
    httpd_register_uri_handler(server, &root_uri);
    httpd_register_uri_handler(server, &favicon_uri);
    httpd_register_uri_handler(server, &api_temp_uri);
    httpd_register_uri_handler(server, &inc_uri);
    httpd_register_uri_handler(server, &dec_uri);
    httpd_register_uri_handler(server, &ota_uri);
}

void app_main(void)
{
    onewire_bus_handle_t bus;
    onewire_bus_config_t bus_config = {
        .bus_gpio_num = DS18B20_GPIO,
        .flags.en_pull_up = true,
    };
    onewire_bus_rmt_config_t rmt_config = {
        .max_rx_bytes = 10,
    };
    ds18b20_device_handle_t sensors[MAX_DS18B20_SENSORS];
    int sensor_count = 0;
    onewire_device_iter_handle_t iter = NULL;
    onewire_device_t device;
    esp_err_t search_result;

    g_data_mutex = xSemaphoreCreateMutex();

    ethernet_init();
    start_telnet_server();
    start_webserver();

    ESP_ERROR_CHECK(onewire_new_bus_rmt(&bus_config, &rmt_config, &bus));
    ESP_LOGI(TAG, "1-Wire bus initialized on GPIO%d", DS18B20_GPIO);

    ESP_ERROR_CHECK(onewire_new_device_iter(bus, &iter));
    do {
        search_result = onewire_device_iter_get_next(iter, &device);
        if (search_result == ESP_OK && sensor_count < MAX_DS18B20_SENSORS &&
            ds18b20_new_device_from_enumeration(&device, &(ds18b20_config_t){},
                                                &sensors[sensor_count]) == ESP_OK) {
            ESP_ERROR_CHECK(ds18b20_set_resolution(sensors[sensor_count], DS18B20_RESOLUTION_12B));
            sensor_count++;
        }
    } while (search_result != ESP_ERR_NOT_FOUND);
    ESP_ERROR_CHECK(onewire_del_device_iter(iter));
    ESP_ERROR_CHECK(sensor_count > 0 ? ESP_OK : ESP_ERR_NOT_FOUND);
    ESP_LOGI(TAG, "%d DS18B20 sensor%s found", sensor_count, sensor_count == 1 ? "" : "s");

    float temp_sum_f[MAX_DS18B20_SENSORS] = { 0 };
    int sample_count = 0;

    while (1) {
        ESP_ERROR_CHECK(ds18b20_trigger_temperature_conversion_for_all(bus));
        float temp_f[MAX_DS18B20_SENSORS];

        xSemaphoreTake(g_data_mutex, portMAX_DELAY);
        float offset_f = g_offset_f;
        xSemaphoreGive(g_data_mutex);

        for (int sensor_index = 0; sensor_index < sensor_count; sensor_index++) {
            float temp_c;
            ESP_ERROR_CHECK(ds18b20_get_temperature(sensors[sensor_index], &temp_c));
            temp_f[sensor_index] = temp_c * 9.0f / 5.0f + 32.0f + offset_f;
            temp_sum_f[sensor_index] += temp_f[sensor_index];
        }

        sample_count++;

        if (sample_count >= SAMPLES_PER_AVERAGE) {
            float temp_avg_f[MAX_DS18B20_SENSORS];
            for (int sensor_index = 0; sensor_index < sensor_count; sensor_index++) {
                temp_avg_f[sensor_index] = temp_sum_f[sensor_index] / sample_count;
            }
            ESP_LOGI(TAG, "Room Temp. 1 (avg) = %.2f F", temp_avg_f[0]);
            if (sensor_count > 1) {
                ESP_LOGI(TAG, "Room Temp. 2 (avg) = %.2f F", temp_avg_f[1]);
            }

            xSemaphoreTake(g_data_mutex, portMAX_DELAY);
            g_temp_f = temp_avg_f[0];
            g_temp2_f = sensor_count > 1 ? temp_avg_f[1] : 0.0f;
            xSemaphoreGive(g_data_mutex);

            for (int sensor_index = 0; sensor_index < sensor_count; sensor_index++) {
                temp_sum_f[sensor_index] = 0.0f;
            }
            sample_count = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_INTERVAL_MS));
    }
}
